from __future__ import annotations

import struct
import unittest

from pal_analyzer import analyze_pal_field, analyze_pal_line
from peak67_analyzer import analyze_peak67, analyze_peak67_power_on
from rx7500_protocol import (
    ACQ_EDGE, ACQ_RAW, FLAG_TRUNCATED, Capture, EdgeEvent, PacketParser,
    TRIGGER_FALLING, analyze_capture, build_config_command, build_test_packet,
    build_v2_packet,
)


def make_spi_capture(frame: int) -> Capture:
    samples = [0b1000] * 20
    for bit_index in range(23, -1, -1):
        data = ((frame >> bit_index) & 1) << 1
        samples.extend([0b1000 | data] * 4)
        samples.extend([0b1001 | data] * 4)
        samples.extend([0b1000 | data] * 2)
    samples.extend([0b1100] * 5)
    samples.extend([0b1000] * 20)
    return Capture(number=7, sample_rate_hz=10_000_000, samples=samples, flags=3)


def make_pal_line(*, sync=24, back=29, active=260, front=7, active_code=41) -> Capture:
    line = [0] * sync + [19] * back + [active_code] * active + [19] * front
    return Capture(number=2, sample_rate_hz=5_000_000, samples=line * 3,
                   version=2, acquisition=ACQ_RAW, channel_count=8)


def make_field_capture(*, missing=False, bad_sync=False, truncated=False) -> Capture:
    pulses: list[tuple[int, int]] = []
    for field in (0, 20_000, 40_000):
        for index in range(15):
            width = 27 if 5 <= index < 10 else 2
            if bad_sync and field == 0 and index == 0:
                width = 12
            if missing and field == 20_000 and index == 7:
                continue
            pulses.append((field + index * 32, width))
        start = field + 480
        end = field + 20_000
        while start < end:
            pulses.append((start, 5))
            start += 64
    events: list[EdgeEvent] = []
    state = 1
    for start, width in sorted(pulses):
        state &= ~1
        events.append(EdgeEvent(start, state, 1))
        state |= 1
        events.append(EdgeEvent(start + width, state, 1))
    return Capture(number=3, sample_rate_hz=0, version=2, acquisition=ACQ_EDGE,
                   channel_count=8, timestamp_hz=1_000_000, duration_ticks=45_000,
                   flags=FLAG_TRUNCATED if truncated else 0, events=events)


def make_three_wire_capture(value: int, bit_count: int, *, lsb_first: bool,
                            latch_pulse: bool) -> Capture:
    bits = [((value >> index) & 1) for index in range(bit_count)] if lsb_first else [
        ((value >> index) & 1) for index in range(bit_count - 1, -1, -1)
    ]
    state = (bits[0] << 1) | (0 if latch_pulse else 0)
    events: list[EdgeEvent] = []
    for index, bit in enumerate(bits):
        base = index * 10
        if index and bool(state & 0b10) != bool(bit):
            state ^= 0b10
            events.append(EdgeEvent(base - 3, state, 0b10))
        state |= 0b001
        events.append(EdgeEvent(base, state, 0b001))
        state &= ~0b001
        events.append(EdgeEvent(base + 4, state, 0b001))
    after = bit_count * 10
    state |= 0b100
    events.append(EdgeEvent(after, state, 0b100))
    if latch_pulse:
        state &= ~0b100
        events.append(EdgeEvent(after + 5, state, 0b100))
    return Capture(number=11, sample_rate_hz=0, version=2, acquisition=ACQ_EDGE,
                   channel_count=8, timestamp_hz=1_000_000,
                   initial_state=(bits[0] << 1), duration_ticks=after + 20,
                   events=events)


def make_power_on_capture(value: int, power_to_cs_us: int = 500) -> Capture:
    state = 0
    events: list[EdgeEvent] = []

    state |= 1 << 3
    events.append(EdgeEvent(0, state, 1 << 3))
    state |= 0b011
    events.append(EdgeEvent(100, state, 0b011))
    state &= ~0b010
    events.append(EdgeEvent(power_to_cs_us, state, 0b010))

    for offset, bit_index in enumerate(range(31, -1, -1)):
        base = power_to_cs_us + 1 + offset * 6
        state &= ~0b001
        events.append(EdgeEvent(base, state, 0b001))
        bit = (value >> bit_index) & 1
        if bool(state & 0b100) != bool(bit):
            state ^= 0b100
            events.append(EdgeEvent(base + 1, state, 0b100))
        state |= 0b001
        events.append(EdgeEvent(base + 2, state, 0b001))

    state |= 0b010
    events.append(EdgeEvent(power_to_cs_us + 194, state, 0b010))
    return Capture(number=12, sample_rate_hz=0, version=2,
                   acquisition=ACQ_EDGE, channel_count=8,
                   timestamp_hz=1_000_000, initial_state=0,
                   duration_ticks=1_000_000, trigger_channel=3,
                   events=events)


def remap_three_lines(capture: Capture, destination_for_source: tuple[int, int, int]) -> Capture:
    def remap(value: int) -> int:
        result = value & ~0b111
        for source, destination in enumerate(destination_for_source):
            if value & (1 << source):
                result |= 1 << destination
        return result
    return Capture(number=capture.number, sample_rate_hz=0, version=2,
                   acquisition=ACQ_EDGE, channel_count=8,
                   timestamp_hz=capture.timestamp_hz,
                   initial_state=remap(capture.initial_state),
                   duration_ticks=capture.duration_ticks,
                   events=[EdgeEvent(event.timestamp_ticks, remap(event.state),
                                     remap(event.changed_mask))
                           for event in capture.events])


class ProtocolTests(unittest.TestCase):
    def test_v1_fragmentation_boot_text_and_rx7500(self):
        source = make_spi_capture(0x011806)
        packet = b"ESP-ROM:esp32s3\r\n" + build_test_packet(source)
        parser = PacketParser()
        decoded = []
        for offset in range(0, len(packet), 37):
            decoded.extend(parser.feed(packet[offset:offset + 37]))
        self.assertEqual(len(decoded), 1)
        self.assertEqual(decoded[0].samples, source.samples)
        analysis = analyze_capture(decoded[0])
        self.assertEqual(analysis["frame"], 0x011806)
        self.assertEqual(analysis["direct_frequency"], 6150)
        self.assertEqual(len(analysis["clk_rising"]), 24)

    def test_v1_odd_sample_count(self):
        source = Capture(1, 1_000_000, [1, 2, 3])
        decoded = PacketParser().feed(build_test_packet(source))[0]
        self.assertEqual(decoded.samples, source.samples)

    def test_v2_raw_8_channels(self):
        source = Capture(8, 5_000_000, [0x00, 0x55, 0xAA, 0xFF], version=2,
                         acquisition=ACQ_RAW, channel_count=8, timestamp_hz=240_000_000,
                         request_id=9, trigger_channel=4, trigger_edge=TRIGGER_FALLING,
                         duration_ticks=192)
        packet = build_v2_packet(source)
        parser = PacketParser()
        decoded = parser.feed(packet[:11]) + parser.feed(packet[11:])
        self.assertEqual(decoded[0].samples, source.samples)
        self.assertEqual(decoded[0].channel_count, 8)
        self.assertEqual(decoded[0].request_id, 9)

    def test_v2_edge_events(self):
        source = make_field_capture()
        decoded = PacketParser().feed(build_v2_packet(source))[0]
        self.assertEqual(decoded.events, source.events)
        self.assertEqual(decoded.duration_ticks, 45_000)

    def test_crc_recovery_and_multiple_packets(self):
        good = build_v2_packet(make_field_capture())
        bad = bytearray(good)
        bad[-1] ^= 0x40
        parser = PacketParser()
        result = parser.feed(bytes(bad) + b"junk" + good + good)
        self.assertEqual(len(result), 2)
        self.assertEqual(parser.crc_errors, 1)

    def test_config_command_crc_and_size(self):
        command = build_config_command(request_id=5, acquisition=ACQ_RAW,
                                       sample_rate_hz=5_000_000, sample_count=8192,
                                       trigger_channel=0, trigger_edge=TRIGGER_FALLING)
        self.assertEqual(len(command), 36)
        self.assertEqual(command[:4], b"LAC2")
        self.assertNotEqual(struct.unpack_from("<I", command, 32)[0], 0)


class PalTests(unittest.TestCase):
    def test_normal_pal_line_and_dac_codes(self):
        result = analyze_pal_line(make_pal_line())
        self.assertEqual(result["verdict"], "PASS")
        self.assertEqual(result["symbolic_codes"], [(0, "SYNC"), (19, "BLACK"), (41, "GRAY TEST")])
        self.assertAlmostEqual(result["measured_us"]["line"], 64.0)

    def test_wrong_hsync_width(self):
        result = analyze_pal_line(make_pal_line(sync=34, back=19))
        self.assertEqual(result["ratings"]["hsync"], "FAIL")

    def test_wrong_line_period(self):
        capture = make_pal_line(front=32)
        result = analyze_pal_line(capture)
        self.assertEqual(result["verdict"], "FAIL")

    def test_half_line_vertical_sequence_and_second_field_phase(self):
        result = analyze_pal_field(make_field_capture())
        self.assertEqual(result["verdict"], "PASS")
        self.assertTrue(result["half_line_periods_us"])
        self.assertAlmostEqual(result["field_periods_us"][0], 20_000.0)
        self.assertAlmostEqual(result["frame_periods_us"][0], 40_000.0)
        self.assertAlmostEqual(result["second_field_phase_us"], 32.0)
        self.assertTrue(all(sequence["broad"] == 5 for sequence in result["sequences"]))

    def test_missing_pulse(self):
        result = analyze_pal_field(make_field_capture(missing=True))
        self.assertTrue(any(sequence["broad"] != 5 for sequence in result["sequences"]))
        self.assertNotEqual(result["verdict"], "PASS")

    def test_bad_sync_width(self):
        result = analyze_pal_field(make_field_capture(bad_sync=True))
        self.assertGreater(result["anomalous_count"], 0)

    def test_truncated_capture(self):
        self.assertTrue(analyze_pal_field(make_field_capture(truncated=True))["truncated"])

    def test_timestamp_wraparound_delta_is_supported_by_packet(self):
        capture = Capture(4, 0, version=2, acquisition=ACQ_EDGE, channel_count=8,
                          timestamp_hz=240_000_000, duration_ticks=100,
                          events=[EdgeEvent(0xFFFFFFF0, 0, 1), EdgeEvent(0x10, 1, 1)])
        decoded = PacketParser().feed(build_v2_packet(capture))[0]
        self.assertEqual(decoded.events[1].timestamp_ticks, 0x10)


class Peak67ReverseEngineeringTests(unittest.TestCase):
    def test_power_on_timing_survives_lost_clock_edges(self):
        events = [
            EdgeEvent(0, 0b1000, 0b1000),
            EdgeEvent(50_000, 0b1011, 0b0011),
            EdgeEvent(100_000, 0b1001, 0b0010),
            EdgeEvent(100_002, 0b1000, 0b0001),
            EdgeEvent(100_003, 0b1001, 0b0001),
            EdgeEvent(100_195, 0b1011, 0b0010),
        ]
        capture = Capture(number=13, sample_rate_hz=0, version=2,
                          acquisition=ACQ_EDGE, channel_count=8,
                          timestamp_hz=1_000_000, initial_state=0,
                          duration_ticks=1_000_000, trigger_channel=3,
                          events=events)
        result = analyze_peak67_power_on(capture)
        command = result["first_command_window"]
        self.assertEqual(result["power_timing_verdict"],
                         "COMMAND TIMING FOUND; EDGE DATA INCOMPLETE")
        self.assertEqual(command["power_to_cs_fall_us"], 100_000.0)
        self.assertEqual(command["power_to_frame_complete_us"], 100_195.0)
        self.assertEqual(command["captured_clk_rising_count"], 1)

    def test_power_on_delay_to_first_complete_frame(self):
        result = analyze_peak67_power_on(make_power_on_capture(0x00580005))
        frame = result["first_valid_frame"]
        self.assertEqual(result["power_timing_verdict"], "VALID 32-BIT FRAME FOUND")
        self.assertEqual(result["power_rise_us"], 0.0)
        self.assertEqual(frame["frame"], 0x00580005)
        self.assertEqual(frame["power_to_cs_fall_us"], 500.0)
        self.assertEqual(frame["power_to_first_clk_rise_us"], 503.0)
        self.assertEqual(frame["power_to_frame_complete_us"], 694.0)

    def test_24_bit_msb_frame_and_positive_latch(self):
        result = analyze_peak67(make_three_wire_capture(
            0xA5123C, 24, lsb_first=False, latch_pulse=True))
        self.assertEqual(result["verdict"], "CAPTURED")
        self.assertEqual(result["clk_rising_count"], 24)
        self.assertEqual(result["frames"][0]["rising"]["msb_value"], 0xA5123C)
        self.assertIn("LE/latch", result["control_role_candidate"])

    def test_25_bit_lsb_frame_and_active_low_cs(self):
        value = 0x1234567
        result = analyze_peak67(make_three_wire_capture(
            value, 25, lsb_first=True, latch_pulse=False))
        self.assertEqual(result["frames"][0]["rising"]["bit_count"], 25)
        self.assertEqual(result["frames"][0]["rising"]["lsb_value"], value)
        self.assertIn("active-LOW CS", result["control_role_candidate"])

    def test_unknown_wire_order_is_detected(self):
        source = make_three_wire_capture(0x5A3C, 16, lsb_first=False, latch_pulse=True)
        # Original CLK/DATA/LE on CH0/CH1/CH2 become CH1/CH2/CH0.
        result = analyze_peak67(remap_three_lines(source, (1, 2, 0)))
        self.assertEqual(result["detected_clock_channel"], 1)
        self.assertEqual(result["detected_data_channel"], 2)
        self.assertEqual(result["detected_control_channel"], 0)
        self.assertEqual(result["frames"][0]["rising"]["msb_value"], 0x5A3C)


if __name__ == "__main__":
    unittest.main(verbosity=2)
