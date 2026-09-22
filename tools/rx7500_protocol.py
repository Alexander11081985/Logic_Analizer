"""Versioned transport and generic capture helpers for the ESP32-S3 analyzer."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import struct
import zlib

MAGIC = b"RX75"
COMMAND_MAGIC = b"LAC2"
VERSION_V1 = 1
VERSION_V2 = 2

V1_HEADER = struct.Struct("<4sBBHIIIII")
V1_HEADER_SIZE = V1_HEADER.size
V2_HEADER = struct.Struct("<4sBBHIII6B H 7I")
V2_HEADER_SIZE = V2_HEADER.size
COMMAND_V2 = struct.Struct("<4sBBHIBBBBIIIII")
COMMAND_V2_SIZE = COMMAND_V2.size
EDGE_EVENT = struct.Struct("<IBBH")

MAX_PAYLOAD_SIZE = 256 * 1024
MAX_SAMPLE_COUNT = 32768
MAX_EDGE_EVENTS = 8192

ACQ_RAW = 1
ACQ_EDGE = 2
PACKING_U8 = 1
PACKING_EDGE8 = 2

TRIGGER_RISING = 0
TRIGGER_FALLING = 1
TRIGGER_EITHER = 2
TRIGGER_IMMEDIATE = 3
TRIGGER_CHANNEL_IMMEDIATE = 0xFF

FLAG_TRIGGERED = 1 << 0
FLAG_TIMEOUT = 1 << 1
FLAG_OVERFLOW = 1 << 2
FLAG_TRUNCATED = 1 << 3

CLK = 1 << 0
DATA = 1 << 1
LE = 1 << 2
MODE = 1 << 3


@dataclass(slots=True)
class EdgeEvent:
    timestamp_ticks: int
    state: int
    changed_mask: int


@dataclass(slots=True)
class Capture:
    number: int
    sample_rate_hz: int
    samples: list[int] = field(default_factory=list)
    flags: int = 0
    version: int = VERSION_V1
    request_id: int = 0
    timestamp_hz: int = 0
    acquisition: int = ACQ_RAW
    channel_count: int = 4
    packing: int = 0
    trigger_channel: int = 0
    trigger_edge: int = TRIGGER_RISING
    initial_state: int = 0
    status: int = 0
    duration_ticks: int = 0
    lost_events: int = 0
    trigger_timeout_ms: int = 0
    item_count: int = 0
    events: list[EdgeEvent] = field(default_factory=list)

    @property
    def sample_period_ns(self) -> float:
        return 1_000_000_000.0 / self.sample_rate_hz if self.sample_rate_hz else 0.0

    @property
    def duration_us(self) -> float:
        if self.acquisition == ACQ_EDGE:
            return self.duration_ticks * 1_000_000.0 / self.timestamp_hz if self.timestamp_hz else 0.0
        return len(self.samples) * 1_000_000.0 / self.sample_rate_hz if self.sample_rate_hz else 0.0


class PacketParser:
    """Streaming v1/v2 parser; tolerates boot text, fragmentation and damage."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.header_errors = 0

    def feed(self, data: bytes) -> list[Capture]:
        self.buffer.extend(data)
        captures: list[Capture] = []
        while True:
            magic_index = self.buffer.find(MAGIC)
            if magic_index < 0:
                keep = min(len(self.buffer), len(MAGIC) - 1)
                if len(self.buffer) > keep:
                    del self.buffer[:-keep]
                break
            if magic_index:
                del self.buffer[:magic_index]
            if len(self.buffer) < 8:
                break

            version = self.buffer[4]
            header_size = struct.unpack_from("<H", self.buffer, 6)[0]
            expected = V1_HEADER_SIZE if version == VERSION_V1 else V2_HEADER_SIZE if version == VERSION_V2 else 0
            if header_size != expected:
                self.header_errors += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < header_size:
                break

            parsed = self._parse_v1_header() if version == VERSION_V1 else self._parse_v2_header()
            if parsed is None:
                self.header_errors += 1
                del self.buffer[0]
                continue
            payload_size, expected_crc, capture = parsed
            packet_size = header_size + payload_size
            if len(self.buffer) < packet_size:
                break
            payload = bytes(self.buffer[header_size:packet_size])
            if (zlib.crc32(payload) & 0xFFFFFFFF) != expected_crc:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            self._decode_payload(capture, payload)
            captures.append(capture)
            del self.buffer[:packet_size]
        return captures

    def _parse_v1_header(self) -> tuple[int, int, Capture] | None:
        (magic, version, flags, header_size, number, rate, count,
         payload_size, crc) = V1_HEADER.unpack_from(self.buffer)
        if not (magic == MAGIC and version == VERSION_V1 and header_size == V1_HEADER_SIZE
                and 0 < rate <= 100_000_000 and 0 < count <= 1_000_000
                and payload_size == (count + 1) // 2 and payload_size <= MAX_PAYLOAD_SIZE):
            return None
        return payload_size, crc, Capture(number=number, sample_rate_hz=rate,
                                           flags=flags, version=VERSION_V1,
                                           channel_count=4, item_count=count)

    def _parse_v2_header(self) -> tuple[int, int, Capture] | None:
        values = V2_HEADER.unpack_from(self.buffer)
        (magic, version, flags, header_size, number, request_id, timestamp_hz,
         acquisition, channels, packing, trigger_channel, trigger_edge,
         initial_state, status, sample_rate, item_count, duration_ticks,
         payload_size, lost_events, timeout_ms, crc) = values
        valid_common = (magic == MAGIC and version == VERSION_V2
                        and header_size == V2_HEADER_SIZE and channels == 8
                        and timestamp_hz > 0 and trigger_channel in (*range(8), 0xFF)
                        and trigger_edge <= TRIGGER_IMMEDIATE
                        and payload_size <= MAX_PAYLOAD_SIZE)
        valid_raw_items = ((0 < item_count <= MAX_SAMPLE_COUNT) or
                           (item_count == 0 and bool(flags & FLAG_TIMEOUT)))
        valid_raw = (acquisition == ACQ_RAW and packing == PACKING_U8
                     and 0 < sample_rate <= 20_000_000
                     and valid_raw_items and payload_size == item_count)
        valid_edge = (acquisition == ACQ_EDGE and packing == PACKING_EDGE8
                      and sample_rate == 0 and item_count <= MAX_EDGE_EVENTS
                      and payload_size == item_count * EDGE_EVENT.size)
        if not (valid_common and (valid_raw or valid_edge)):
            return None
        capture = Capture(
            number=number, sample_rate_hz=sample_rate, flags=flags,
            version=VERSION_V2, request_id=request_id, timestamp_hz=timestamp_hz,
            acquisition=acquisition, channel_count=channels, packing=packing,
            trigger_channel=trigger_channel, trigger_edge=trigger_edge,
            initial_state=initial_state, status=status,
            duration_ticks=duration_ticks, lost_events=lost_events,
            trigger_timeout_ms=timeout_ms,
            item_count=item_count,
        )
        return payload_size, crc, capture

    @staticmethod
    def _decode_payload(capture: Capture, payload: bytes) -> None:
        if capture.version == VERSION_V1:
            for packed in payload:
                capture.samples.extend((packed & 0x0F, (packed >> 4) & 0x0F))
            del capture.samples[capture.item_count:]
        elif capture.acquisition == ACQ_RAW:
            capture.samples = list(payload)
        else:
            capture.events = [EdgeEvent(t, state, changed)
                              for t, state, changed, _ in EDGE_EVENT.iter_unpack(payload)]


def build_test_packet(capture: Capture) -> bytes:
    """Build the legacy v1 packet used by RX7500 regression tests."""
    payload = bytearray((len(capture.samples) + 1) // 2)
    for index in range(0, len(capture.samples), 2):
        first = capture.samples[index] & 0x0F
        second = capture.samples[index + 1] & 0x0F if index + 1 < len(capture.samples) else 0
        payload[index // 2] = first | (second << 4)
    return V1_HEADER.pack(MAGIC, VERSION_V1, capture.flags, V1_HEADER_SIZE,
                          capture.number, capture.sample_rate_hz, len(capture.samples),
                          len(payload), zlib.crc32(payload) & 0xFFFFFFFF) + payload


def build_v2_packet(capture: Capture) -> bytes:
    if capture.acquisition == ACQ_RAW:
        payload = bytes(sample & 0xFF for sample in capture.samples)
        packing = PACKING_U8
        item_count = len(capture.samples)
    else:
        payload = b"".join(EDGE_EVENT.pack(event.timestamp_ticks, event.state,
                                           event.changed_mask, 0)
                           for event in capture.events)
        packing = PACKING_EDGE8
        item_count = len(capture.events)
    header = V2_HEADER.pack(
        MAGIC, VERSION_V2, capture.flags, V2_HEADER_SIZE, capture.number,
        capture.request_id, capture.timestamp_hz, capture.acquisition, 8,
        packing, capture.trigger_channel, capture.trigger_edge,
        capture.initial_state, capture.status, capture.sample_rate_hz,
        item_count, capture.duration_ticks, len(payload), capture.lost_events,
        capture.trigger_timeout_ms, zlib.crc32(payload) & 0xFFFFFFFF,
    )
    return header + payload


def build_config_command(*, request_id: int, acquisition: int,
                         sample_rate_hz: int = 0, sample_count: int = 0,
                         duration_us: int = 0, trigger_channel: int = 0,
                         trigger_edge: int = TRIGGER_RISING,
                         trigger_timeout_ms: int = 1000) -> bytes:
    prefix = COMMAND_V2.pack(COMMAND_MAGIC, VERSION_V2, 1, COMMAND_V2_SIZE,
                             request_id, acquisition, trigger_channel,
                             trigger_edge, 0, sample_rate_hz, sample_count,
                             duration_us, trigger_timeout_ms, 0)
    crc = zlib.crc32(prefix[:-4]) & 0xFFFFFFFF
    return prefix[:-4] + struct.pack("<I", crc)


def edge_indices(samples: list[int], mask: int) -> tuple[list[int], list[int]]:
    rising: list[int] = []
    falling: list[int] = []
    if samples and samples[0] & mask:
        rising.append(0)
    for index in range(1, len(samples)):
        previous = bool(samples[index - 1] & mask)
        current = bool(samples[index] & mask)
        if not previous and current:
            rising.append(index)
        elif previous and not current:
            falling.append(index)
    return rising, falling


def transition_rows(samples: list[int], channel_mask: int = 0xFF) -> list[tuple[int, int, int]]:
    if not samples:
        return []
    rows = [(0, samples[0], channel_mask)]
    for index in range(1, len(samples)):
        changed = (samples[index - 1] ^ samples[index]) & channel_mask
        if changed:
            rows.append((index, samples[index], changed))
    return rows


def capture_transition_rows(capture: Capture) -> list[tuple[int, int, int]]:
    if capture.acquisition == ACQ_EDGE:
        return [(event.timestamp_ticks, event.state, event.changed_mask)
                for event in capture.events]
    return transition_rows(capture.samples, (1 << capture.channel_count) - 1)


def suppress_short_pulses(samples: list[int], minimum_samples: int,
                          channel_count: int = 8) -> list[int]:
    if minimum_samples <= 1 or not samples:
        return list(samples)
    result = list(samples)
    for bit in range(channel_count):
        mask = 1 << bit
        levels = [1 if sample & mask else 0 for sample in result]
        for _ in range(8):
            runs: list[tuple[int, int, int]] = []
            start = 0
            for index in range(1, len(levels) + 1):
                if index == len(levels) or levels[index] != levels[start]:
                    runs.append((start, index, levels[start]))
                    start = index
            changed_any = False
            for run_index in range(1, len(runs) - 1):
                start, end, level = runs[run_index]
                before, after = runs[run_index - 1][2], runs[run_index + 1][2]
                if end - start < minimum_samples and before == after != level:
                    levels[start:end] = [before] * (end - start)
                    changed_any = True
            if not changed_any:
                break
        for index, level in enumerate(levels):
            result[index] = (result[index] | mask) if level else (result[index] & ~mask)
    return result


def _stats(values: list[int], sample_period_us: float) -> tuple[float | None, float | None, float | None]:
    if not values:
        return None, None, None
    converted = [value * sample_period_us for value in values]
    return min(converted), sum(converted) / len(converted), max(converted)


def analyze_capture(capture: Capture, glitch_filter_ns: float = 0.0) -> dict[str, object]:
    if capture.acquisition != ACQ_RAW:
        raise ValueError("RX7500 decoder requires raw samples")
    minimum_samples = max(1, math.ceil(glitch_filter_ns / capture.sample_period_ns))
    samples = suppress_short_pulses(capture.samples, minimum_samples, capture.channel_count)
    sample_period_us = capture.sample_period_ns / 1000.0
    clk_rising, clk_falling = edge_indices(samples, CLK)
    le_rising, le_falling = edge_indices(samples, LE)
    transitions = transition_rows(samples, (1 << capture.channel_count) - 1)
    bits: list[int] = []
    frame = 0
    for edge in clk_rising[:24]:
        bit = 1 if samples[edge] & DATA else 0
        bits.append(bit)
        frame = (frame << 1) | bit
    address = (frame >> 16) & 0xFF if len(bits) == 24 else None
    value = frame & 0xFFFF if len(bits) == 24 else None
    frequency = value if address == 0x01 and 6000 <= value <= 7500 else None
    periods = [clk_rising[i] - clk_rising[i - 1] for i in range(2, len(clk_rising))]
    high_widths = []
    for rising in clk_rising[1:]:
        falling = next((edge for edge in clk_falling if edge > rising), None)
        if falling is not None:
            high_widths.append(falling - rising)
    le_pulses = []
    for rise in le_rising:
        fall = next((edge for edge in le_falling if edge > rise), None)
        if fall is not None:
            le_pulses.append((rise, fall))
    selected_le = None
    if le_pulses:
        if len(clk_rising) >= 24:
            selected_le = next((pulse for pulse in le_pulses if pulse[0] >= clk_rising[23]), None)
        selected_le = selected_le or le_pulses[0]
    setup_samples = []
    for edge in clk_rising[1:24]:
        for index in range(edge, 0, -1):
            if (samples[index] ^ samples[index - 1]) & DATA:
                setup_samples.append(edge - index)
                break
    mode_high = sum(1 for sample in samples if sample & MODE)
    return {
        "samples": samples, "transitions": transitions,
        "clk_rising": clk_rising, "clk_falling": clk_falling,
        "le_pulses": le_pulses, "selected_le": selected_le, "bits": bits,
        "frame": frame if len(bits) == 24 else None, "address": address,
        "value": value, "direct_frequency": frequency,
        "frame_edge_count_valid": len(clk_rising) == 24,
        "period_stats_us": _stats(periods, sample_period_us),
        "high_stats_us": _stats(high_widths, sample_period_us),
        "byte_gap_8_9_us": ((clk_rising[8] - clk_rising[7]) * sample_period_us if len(clk_rising) >= 9 else None),
        "byte_gap_16_17_us": ((clk_rising[16] - clk_rising[15]) * sample_period_us if len(clk_rising) >= 17 else None),
        "minimum_setup_us": min(setup_samples) * sample_period_us if setup_samples else None,
        "mode_high_percent": 100.0 * mode_high / len(samples) if samples else 0.0,
        "sample_period_us": sample_period_us,
        "changed_names": {1 << bit: ("CLK", "DATA", "LE", "SPI/M")[bit] if bit < 4 else f"AUX{bit}" for bit in range(capture.channel_count)},
        "glitch_filter_samples": minimum_samples,
    }
