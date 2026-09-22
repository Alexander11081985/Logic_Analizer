"""Independent PAL GPIO-DAC, sync-field and failover analysis."""

from __future__ import annotations

from dataclasses import dataclass
from statistics import mean

from rx7500_protocol import ACQ_EDGE, ACQ_RAW, Capture, EdgeEvent

SYNC = 0
BLACK = 19
GRAY = 41
WHITE = 63
SYMBOLIC_LEVELS = {SYNC: "SYNC", BLACK: "BLACK", GRAY: "GRAY TEST", WHITE: "WHITE"}

PAL_LINE_US = 64.0
PAL_HSYNC_US = 4.7
PAL_BACK_PORCH_US = 5.8
PAL_ACTIVE_US = 52.0
PAL_FRONT_PORCH_US = 1.5
PAL_FIELD_US = 20_000.0
PAL_FRAME_US = 40_000.0
PAL_HALF_LINE_US = 32.0

TOLERANCES = {
    "line": (1.0, 2.0),
    "hsync": (0.5, 1.0),
    "back_porch": (1.0, 2.0),
    "active": (1.5, 3.0),
    "front_porch": (0.6, 1.2),
    "half_line": (1.0, 2.0),
    "field": (150.0, 400.0),
    "frame": (250.0, 600.0),
}


def _rating(error: float, key: str) -> str:
    passed, warned = TOLERANCES[key]
    return "PASS" if abs(error) <= passed else "WARN" if abs(error) <= warned else "FAIL"


def _overall(ratings: list[str]) -> str:
    if "FAIL" in ratings:
        return "FAIL"
    if "WARN" in ratings:
        return "WARN"
    return "PASS"


def dac_runs(samples: list[int]) -> list[tuple[int, int, int]]:
    if not samples:
        return []
    codes = [sample & 0x3F for sample in samples]
    runs: list[tuple[int, int, int]] = []
    start = 0
    for index in range(1, len(codes) + 1):
        if index == len(codes) or codes[index] != codes[start]:
            runs.append((start, index, codes[start]))
            start = index
    return runs


def analyze_pal_line(capture: Capture) -> dict[str, object]:
    if capture.acquisition != ACQ_RAW or not capture.samples or not capture.sample_rate_hz:
        return {"verdict": "FAIL", "error": "PAL line requires raw samples"}
    period_us = 1_000_000.0 / capture.sample_rate_hz
    runs = dac_runs(capture.samples)
    sync_runs = [(start, end) for start, end, code in runs
                 if code == SYNC and 2.5 <= (end - start) * period_us <= 7.5]
    if len(sync_runs) < 2:
        return {"verdict": "FAIL", "error": "two complete H-sync pulses not found",
                "resolution_us": period_us, "runs": runs}
    first = sync_runs[0]
    second = next((run for run in sync_runs[1:] if 55.0 <= (run[0] - first[0]) * period_us <= 72.0), None)
    if second is None:
        return {"verdict": "FAIL", "error": "complete PAL line not found",
                "resolution_us": period_us, "runs": runs}
    line_start, line_end = first[0], second[0]
    sync_end = first[1]
    line_runs = [(max(start, line_start), min(end, line_end), code)
                 for start, end, code in runs if end > line_start and start < line_end]
    nonblack = [(start, end) for start, end, code in line_runs
                if start >= sync_end and code not in (SYNC, BLACK)]
    if nonblack:
        active_start = nonblack[0][0]
        active_end = nonblack[-1][1]
        back_porch = (active_start - sync_end) * period_us
        active = (active_end - active_start) * period_us
        front_porch = (line_end - active_end) * period_us
    else:
        active_start = active_end = None
        back_porch = active = front_porch = None

    measured = {
        "line": (line_end - line_start) * period_us,
        "hsync": (sync_end - line_start) * period_us,
        "back_porch": back_porch,
        "active": active,
        "front_porch": front_porch,
    }
    expected = {"line": PAL_LINE_US, "hsync": PAL_HSYNC_US,
                "back_porch": PAL_BACK_PORCH_US, "active": PAL_ACTIVE_US,
                "front_porch": PAL_FRONT_PORCH_US}
    ratings: dict[str, str] = {}
    errors: dict[str, float | None] = {}
    for key, value in measured.items():
        errors[key] = None if value is None else value - expected[key]
        ratings[key] = "WARN" if value is None else _rating(errors[key], key)
    codes = sorted({code for _, _, code in line_runs})
    return {
        "verdict": _overall(list(ratings.values())), "resolution_us": period_us,
        "line_start_sample": line_start, "line_end_sample": line_end,
        "active_start_sample": active_start, "active_end_sample": active_end,
        "measured_us": measured, "expected_us": expected, "errors_us": errors,
        "ratings": ratings, "codes": codes,
        "symbolic_codes": [(code, SYMBOLIC_LEVELS.get(code, "INTERMEDIATE")) for code in codes],
        "runs": line_runs,
    }


def _edge_times(capture: Capture, channel: int, rising: bool) -> list[int]:
    mask = 1 << channel
    result = []
    for event in capture.events:
        if event.changed_mask & mask and bool(event.state & mask) == rising:
            result.append(event.timestamp_ticks)
    return result


def _to_us(capture: Capture, ticks: int) -> float:
    return ticks * 1_000_000.0 / capture.timestamp_hz


def sync_pulses(capture: Capture, channel: int = 0,
                active_low: bool = True) -> list[dict[str, float | str]]:
    starts = _edge_times(capture, channel, rising=not active_low)
    ends = _edge_times(capture, channel, rising=active_low)
    pulses = []
    end_index = 0
    for start in starts:
        while end_index < len(ends) and ends[end_index] <= start:
            end_index += 1
        if end_index >= len(ends):
            break
        width = _to_us(capture, ends[end_index] - start)
        kind = ("equalizing" if 1.2 <= width < 3.5 else
                "normal" if 3.5 <= width < 8.0 else
                "broad" if 20.0 <= width <= 31.5 else "anomalous")
        pulses.append({"start_ticks": start, "start_us": _to_us(capture, start),
                       "width_us": width, "kind": kind})
        end_index += 1
    return pulses


def analyze_pal_field(capture: Capture, sync_channel: int = 0,
                      active_low: bool = True) -> dict[str, object]:
    if capture.acquisition != ACQ_EDGE or not capture.timestamp_hz:
        return {"verdict": "FAIL", "error": "PAL field requires edge events"}
    pulses = sync_pulses(capture, sync_channel, active_low)
    starts = [float(p["start_us"]) for p in pulses]
    widths = [float(p["width_us"]) for p in pulses]
    cadence = [starts[i] - starts[i - 1] for i in range(1, len(starts))]
    normal_periods = [value for value in cadence if 58.0 <= value <= 70.0]
    half_periods = [value for value in cadence if 28.0 <= value <= 36.0]
    broad_indices = [i for i, pulse in enumerate(pulses) if pulse["kind"] == "broad"]
    groups: list[list[int]] = []
    for index in broad_indices:
        if not groups or starts[index] - starts[groups[-1][-1]] > 80.0:
            groups.append([index])
        else:
            groups[-1].append(index)
    field_starts = [starts[group[0]] for group in groups]
    field_periods = [field_starts[i] - field_starts[i - 1] for i in range(1, len(field_starts))]
    frame_periods = [field_starts[i] - field_starts[i - 2] for i in range(2, len(field_starts))]

    sequences = []
    for group in groups:
        first, last = group[0], group[-1]
        pre = sum(1 for pulse in pulses[max(0, first - 5):first] if pulse["kind"] == "equalizing")
        post = sum(1 for pulse in pulses[last + 1:last + 6] if pulse["kind"] == "equalizing")
        sequences.append({"start_us": starts[first], "pre_equalizing": pre,
                          "broad": len(group), "post_equalizing": post})

    ratings = []
    if normal_periods:
        ratings.append(_rating(mean(normal_periods) - PAL_LINE_US, "line"))
    else:
        ratings.append("FAIL")
    if half_periods:
        ratings.append(_rating(mean(half_periods) - PAL_HALF_LINE_US, "half_line"))
    if field_periods:
        ratings.append(_rating(mean(field_periods) - PAL_FIELD_US, "field"))
    if frame_periods:
        ratings.append(_rating(mean(frame_periods) - PAL_FRAME_US, "frame"))
    if any(s["broad"] != 5 or s["pre_equalizing"] != 5 or s["post_equalizing"] != 5 for s in sequences):
        ratings.append("WARN")

    phase_us = None
    if len(field_starts) >= 2:
        phase_us = (field_starts[1] - field_starts[0]) % PAL_LINE_US
        phase_us = min(phase_us, PAL_LINE_US - phase_us)
    return {
        "verdict": _overall(ratings) if ratings else "FAIL", "pulses": pulses,
        "pulse_widths_us": widths, "normal_line_periods_us": normal_periods,
        "half_line_periods_us": half_periods, "field_periods_us": field_periods,
        "frame_periods_us": frame_periods, "sequences": sequences,
        "second_field_phase_us": phase_us,
        "anomalous_count": sum(p["kind"] == "anomalous" for p in pulses),
        "truncated": bool(capture.flags & (1 << 3)),
    }


def analyze_failover(capture: Capture, *, csync_channel: int = 0,
                     vsync_channel: int = 1, select_channel: int = 2,
                     fake_channel: int = 3, real_select_level: int = 1) -> dict[str, object]:
    if capture.acquisition != ACQ_EDGE or not capture.timestamp_hz:
        return {"verdict": "FAIL", "error": "failover requires edge events"}
    csync = [_to_us(capture, value) for value in _edge_times(capture, csync_channel, False)]
    vsync = [_to_us(capture, value) for value in _edge_times(capture, vsync_channel, False)]
    fake = [_to_us(capture, value) for value in _edge_times(capture, fake_channel, False)]
    periods = [csync[i] - csync[i - 1] for i in range(1, len(csync))]
    valid = [period for period in periods if 60.0 <= period <= 68.0]
    invalid = [period for period in periods if not 60.0 <= period <= 68.0]
    select_mask = 1 << select_channel
    select_events = [event for event in capture.events if event.changed_mask & select_mask]
    to_fake = [_to_us(capture, event.timestamp_ticks) for event in select_events
               if (1 if event.state & select_mask else 0) != real_select_level]
    to_real = [_to_us(capture, event.timestamp_ticks) for event in select_events
               if (1 if event.state & select_mask else 0) == real_select_level]

    gap = next(((csync[i - 1], csync[i]) for i in range(1, len(csync))
                if csync[i] - csync[i - 1] > 1000.0), None)
    last_valid = gap[0] if gap else (csync[-1] if csync else None)
    first_restored = gap[1] if gap else None
    switch_fake = next((value for value in to_fake if last_valid is None or value >= last_valid), None)
    switch_real = next((value for value in to_real if first_restored is None or value >= first_restored), None)
    stable_lock = None
    if first_restored is not None:
        restored_index = next((i for i, value in enumerate(csync) if value >= first_restored), None)
        if restored_index is not None:
            for index in range(restored_index, max(restored_index, len(csync) - 8)):
                if all(60.0 <= csync[j + 1] - csync[j] <= 68.0 for j in range(index, min(index + 8, len(csync) - 1))):
                    stable_lock = csync[index]
                    break
    return {
        "verdict": "PASS" if valid else "WARN", "csync_periods_us": periods,
        "csync_avg_us": mean(valid) if valid else None, "valid_lines": len(valid),
        "invalid_lines": len(invalid),
        "vsync_periods_us": [vsync[i] - vsync[i - 1] for i in range(1, len(vsync))],
        "fake_sync_active": len(fake) >= 2,
        "last_valid_real_sync_us": last_valid,
        "video_sel_to_fake_us": switch_fake,
        "first_restored_sync_us": first_restored,
        "stable_lock_us": stable_lock,
        "video_sel_to_real_us": switch_real,
        "loss_to_switch_us": (switch_fake - last_valid if switch_fake is not None and last_valid is not None else None),
        "restore_to_switch_us": (switch_real - first_restored if switch_real is not None and first_restored is not None else None),
    }
