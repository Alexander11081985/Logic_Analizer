"""Protocol-neutral analysis of the three PEAK67 receiver control lines.

The decoder deliberately reports both clock edges and both bit orders.  Nothing
is labelled as the real PEAK67 protocol until captures from the hardware prove
the clock phase, bit order, frame length and the role/polarity of line 3.
"""

from __future__ import annotations

from statistics import median

from rx7500_protocol import ACQ_EDGE, ACQ_RAW, Capture, suppress_short_pulses, transition_rows

LINE_MASKS = (1 << 0, 1 << 1, 1 << 2)


def _stats(values: list[float]) -> tuple[float | None, float | None, float | None]:
    if not values:
        return None, None, None
    return min(values), sum(values) / len(values), max(values)


def _transitions(capture: Capture, glitch_filter_ns: float) -> tuple[int, list[tuple[float, int, int]]]:
    if capture.acquisition == ACQ_RAW:
        if not capture.samples or not capture.sample_rate_hz:
            return 0, []
        minimum = max(1, int(glitch_filter_ns / capture.sample_period_ns + 0.999999))
        samples = suppress_short_pulses(capture.samples, minimum, capture.channel_count)
        period_us = 1_000_000.0 / capture.sample_rate_hz
        rows = transition_rows(samples, (1 << capture.channel_count) - 1)
        return samples[0], [(index * period_us, state, changed) for index, state, changed in rows]
    if capture.acquisition == ACQ_EDGE:
        if not capture.timestamp_hz:
            return capture.initial_state, []
        scale = 1_000_000.0 / capture.timestamp_hz
        return capture.initial_state, [
            (event.timestamp_ticks * scale, event.state, event.changed_mask)
            for event in capture.events
        ]
    raise ValueError("unsupported acquisition mode")


def _line_edges(rows: list[tuple[float, int, int]], mask: int) -> tuple[list[float], list[float]]:
    rising: list[float] = []
    falling: list[float] = []
    for time_us, state, changed in rows:
        if changed & mask:
            (rising if state & mask else falling).append(time_us)
    return rising, falling


def _pulse_widths(rising: list[float], falling: list[float]) -> tuple[list[float], list[float]]:
    high: list[float] = []
    low: list[float] = []
    fall_index = 0
    for rise in rising:
        while fall_index < len(falling) and falling[fall_index] <= rise:
            fall_index += 1
        if fall_index < len(falling):
            high.append(falling[fall_index] - rise)
    rise_index = 0
    for fall in falling:
        while rise_index < len(rising) and rising[rise_index] <= fall:
            rise_index += 1
        if rise_index < len(rising):
            low.append(rising[rise_index] - fall)
    return high, low


def _sample_bits(rows: list[tuple[float, int, int]], clock_mask: int,
                 data_mask: int, clock_rising: bool) -> list[tuple[float, int, bool]]:
    result: list[tuple[float, int, bool]] = []
    for time_us, state, changed in rows:
        if not changed & clock_mask:
            continue
        is_rising = bool(state & clock_mask)
        if is_rising == clock_rising:
            result.append((time_us, 1 if state & data_mask else 0, bool(changed & data_mask)))
    return result


def _group_clock_edges(edges: list[tuple[float, int, bool]]) -> tuple[list[list[tuple[float, int, bool]]], float | None]:
    if not edges:
        return [], None
    periods = [edges[index][0] - edges[index - 1][0] for index in range(1, len(edges))]
    ordinary = [value for value in periods if value > 0]
    if not ordinary:
        return [edges], None
    base = median(ordinary)
    threshold = max(base * 2.5, base + 1.0)
    groups: list[list[tuple[float, int, bool]]] = [[edges[0]]]
    for previous, current in zip(edges, edges[1:]):
        if current[0] - previous[0] > threshold:
            groups.append([])
        groups[-1].append(current)
    return groups, threshold


def _bits_to_value(bits: list[int], lsb_first: bool) -> int:
    if lsb_first:
        return sum(bit << index for index, bit in enumerate(bits))
    value = 0
    for bit in bits:
        value = (value << 1) | bit
    return value


def _bytes(bits: list[int], lsb_first: bool) -> list[int]:
    result = []
    for offset in range(0, len(bits) - 7, 8):
        result.append(_bits_to_value(bits[offset:offset + 8], lsb_first))
    return result


def _decode_group(group: list[tuple[float, int, bool]]) -> dict[str, object]:
    bits = [bit for _, bit, _ in group]
    width = max(1, (len(bits) + 3) // 4)
    msb_value = _bits_to_value(bits, False)
    lsb_value = _bits_to_value(bits, True)
    return {
        "start_us": group[0][0],
        "end_us": group[-1][0],
        "bit_count": len(bits),
        "bits": bits,
        "bit_string": "".join(str(bit) for bit in bits),
        "msb_value": msb_value,
        "lsb_value": lsb_value,
        "msb_hex": f"0x{msb_value:0{width}X}",
        "lsb_hex": f"0x{lsb_value:0{width}X}",
        "msb_bytes": _bytes(bits, False),
        "lsb_bytes": _bytes(bits, True),
        "simultaneous_data_clock": sum(ambiguous for _, _, ambiguous in group),
    }


def _setup_hold(rows: list[tuple[float, int, int]], data_mask: int,
                sample_edges: list[tuple[float, int, bool]]) -> tuple[list[float], list[float]]:
    data_times = [time_us for time_us, _, changed in rows if changed & data_mask]
    setup: list[float] = []
    hold: list[float] = []
    for edge_time, _, _ in sample_edges:
        previous = [time for time in data_times if time <= edge_time]
        following = [time for time in data_times if time >= edge_time]
        if previous:
            setup.append(edge_time - previous[-1])
        if following:
            hold.append(following[0] - edge_time)
    return setup, hold


def _state_at(initial_state: int, rows: list[tuple[float, int, int]], time_us: float) -> int:
    state = initial_state
    for transition_time, new_state, _ in rows:
        if transition_time > time_us:
            break
        state = new_state
    return state


def _control_role(initial_state: int, final_state: int,
                  rows: list[tuple[float, int, int]],
                  clock_groups: list[list[tuple[float, int, bool]]],
                  control_mask: int) -> str:
    rising, falling = _line_edges(rows, control_mask)
    if not clock_groups:
        return "undetermined: no clock burst"
    last_clock = clock_groups[-1][-1][0]
    next_rise = next((time for time in rising if time >= last_clock), None)
    next_fall = next((time for time in falling if time >= last_clock), None)
    if next_rise is not None:
        fall_after_rise = next((time for time in falling if time > next_rise), None)
        if fall_after_rise is not None:
            return "LE/latch candidate: LOW during clocks, positive pulse after burst"
        if not final_state & control_mask:
            return "positive strobe candidate"
        return "active-LOW CS candidate: line returns HIGH after clock burst"
    if next_fall is not None:
        rise_after_fall = next((time for time in rising if time > next_fall), None)
        if rise_after_fall is not None:
            return "active-HIGH CS or negative latch-pulse candidate"
    if not (initial_state & control_mask) and final_state & control_mask:
        return "active-LOW CS candidate (leading edge occurred before trigger)"
    if initial_state & control_mask and not (final_state & control_mask):
        return "active-HIGH CS candidate (capture ended while active)"
    return "undetermined; change trigger edge or capture another command"


def _detect_mapping(rows: list[tuple[float, int, int]]) -> tuple[int, int, int, str, tuple[int, int, int]]:
    counts = tuple(sum(1 for _, _, changed in rows if changed & mask) for mask in LINE_MASKS)
    clock_channel = max(range(3), key=lambda index: counts[index])
    clock_mask = LINE_MASKS[clock_channel]
    clock_times = [time_us for time_us, _, changed in rows if changed & clock_mask]
    remaining = [index for index in range(3) if index != clock_channel]
    if clock_times:
        last_clock = clock_times[-1]
        after = {
            index: sum(1 for time_us, _, changed in rows
                       if time_us >= last_clock and changed & LINE_MASKS[index])
            for index in remaining
        }
    else:
        after = {index: 0 for index in remaining}
    if after[remaining[0]] != after[remaining[1]]:
        control_channel = max(remaining, key=lambda index: after[index])
        basis = "line with transition after clock burst selected as LE/CS"
    else:
        control_channel = min(remaining, key=lambda index: counts[index])
        basis = "less-active non-clock line selected as LE/CS"
    data_channel = next(index for index in remaining if index != control_channel)
    clock_count = counts[clock_channel]
    confidence = "strong" if clock_count >= max(4, counts[data_channel] * 2) else "tentative"
    if counts[data_channel] == 0:
        confidence = "tentative (DATA may be constant for this command)"
    return clock_channel, data_channel, control_channel, f"{confidence}; {basis}", counts


def analyze_peak67(capture: Capture, glitch_filter_ns: float = 0.0) -> dict[str, object]:
    """Analyze three lines and expose mapping/edge/order candidates."""
    initial_state, rows = _transitions(capture, glitch_filter_ns)
    if not rows:
        return {"verdict": "NO ACTIVITY", "error": "no transitions captured"}
    final_state = rows[-1][1]
    clock_channel, data_channel, control_channel, mapping_basis, transition_counts = _detect_mapping(rows)
    clock_mask = LINE_MASKS[clock_channel]
    data_mask = LINE_MASKS[data_channel]
    control_mask = LINE_MASKS[control_channel]
    clk_rising, clk_falling = _line_edges(rows, clock_mask)
    control_rising, control_falling = _line_edges(rows, control_mask)
    high_widths, low_widths = _pulse_widths(clk_rising, clk_falling)
    control_high, control_low = _pulse_widths(control_rising, control_falling)
    rising_samples = _sample_bits(rows, clock_mask, data_mask, True)
    falling_samples = _sample_bits(rows, clock_mask, data_mask, False)
    rising_groups, gap_threshold = _group_clock_edges(rising_samples)
    falling_groups, _ = _group_clock_edges(falling_samples)
    rising_setup, rising_hold = _setup_hold(rows, data_mask, rising_samples)
    falling_setup, falling_hold = _setup_hold(rows, data_mask, falling_samples)

    periods = [clk_rising[index] - clk_rising[index - 1]
               for index in range(1, len(clk_rising))]
    frames = []
    for index in range(max(len(rising_groups), len(falling_groups))):
        rising = _decode_group(rising_groups[index]) if index < len(rising_groups) else None
        falling = _decode_group(falling_groups[index]) if index < len(falling_groups) else None
        reference = rising or falling
        mid = (reference["start_us"] + reference["end_us"]) / 2.0
        frames.append({
            "index": index + 1,
            "rising": rising,
            "falling": falling,
            "control_level_during_clocks": 1 if _state_at(initial_state, rows, mid) & control_mask else 0,
        })

    unique_rising = {tuple(frame["rising"]["bits"]) for frame in frames if frame["rising"]}
    return {
        "verdict": "CAPTURED" if clk_rising or clk_falling else "NO CLOCK",
        "detected_clock_channel": clock_channel,
        "detected_data_channel": data_channel,
        "detected_control_channel": control_channel,
        "mapping_basis": mapping_basis,
        "transition_counts": transition_counts,
        "initial_levels": tuple(1 if initial_state & (1 << bit) else 0 for bit in range(3)),
        "final_levels": tuple(1 if final_state & (1 << bit) else 0 for bit in range(3)),
        "clk_rising_count": len(clk_rising),
        "clk_falling_count": len(clk_falling),
        "clock_period_stats_us": _stats(periods),
        "clock_high_stats_us": _stats(high_widths),
        "clock_low_stats_us": _stats(low_widths),
        "rising_setup_stats_us": _stats(rising_setup),
        "rising_hold_stats_us": _stats(rising_hold),
        "falling_setup_stats_us": _stats(falling_setup),
        "falling_hold_stats_us": _stats(falling_hold),
        "control_rising_count": len(control_rising),
        "control_falling_count": len(control_falling),
        "control_high_stats_us": _stats(control_high),
        "control_low_stats_us": _stats(control_low),
        "control_role_candidate": _control_role(initial_state, final_state, rows,
                                                  rising_groups or falling_groups,
                                                  control_mask),
        "clock_gap_threshold_us": gap_threshold,
        "frames": frames,
        "unique_rising_frames": len(unique_rising),
        "pretrigger_warning": "first CLK edge is the trigger; DATA setup and line-3 lead time before it are not captured",
    }


def analyze_peak67_power_on(capture: Capture,
                            glitch_filter_ns: float = 0.0) -> dict[str, object]:
    """Measure CH3 3V3 rising to the first complete fixed-mapping frame.

    Fixed wiring: CH0=CLK, CH1=CS, CH2=DATA, CH3=receiver 3V3 sense.  A
    frame is accepted only when one CS LOW window contains exactly 32 CLK
    rising edges; this rejects unrelated power-up transitions.
    """
    result = analyze_peak67(capture, glitch_filter_ns)
    initial_state, rows = _transitions(capture, glitch_filter_ns)
    power_mask = 1 << 3
    clk_mask, cs_mask, data_mask = LINE_MASKS

    power_rises = [time_us for time_us, state, changed in rows
                   if changed & power_mask and state & power_mask]
    power_time = power_rises[0] if power_rises else None
    result.update({
        "power_rise_us": power_time,
        "power_initial_high": bool(initial_state & power_mask),
        "first_valid_frame": None,
        "first_command_window": None,
        "command_windows": [],
        "power_timing_verdict": "NO 3V3 RISING EDGE",
        "pretrigger_warning": (
            "CH3 3V3 rising is t=0. The value is measured from the ESP32 "
            "digital input threshold crossing, not from an ideal 0 V power instant."
        ),
    })
    if power_time is None:
        return result

    cs_falls = [time_us for time_us, state, changed in rows
                if time_us >= power_time and changed & cs_mask
                and not state & cs_mask]
    cs_rises = [time_us for time_us, state, changed in rows
                if time_us >= power_time and changed & cs_mask
                and state & cs_mask]

    for cs_fall in cs_falls:
        cs_rise = next((time_us for time_us in cs_rises if time_us > cs_fall), None)
        if cs_rise is None:
            continue
        clocks = [(time_us, 1 if state & data_mask else 0)
                  for time_us, state, changed in rows
                  if cs_fall <= time_us < cs_rise
                  and changed & clk_mask and state & clk_mask]
        window = {
            "cs_fall_us": cs_fall,
            "first_clk_rise_us": clocks[0][0] if clocks else None,
            "last_clk_rise_us": clocks[-1][0] if clocks else None,
            "cs_rise_us": cs_rise,
            "power_to_cs_fall_us": cs_fall - power_time,
            "power_to_first_clk_rise_us": clocks[0][0] - power_time if clocks else None,
            "power_to_frame_complete_us": cs_rise - power_time,
            "cs_low_us": cs_rise - cs_fall,
            "captured_clk_rising_count": len(clocks),
        }
        # Twelve raw reference captures put a PEAK67 32-bit CS-low frame at
        # approximately 192...196 us.  Edge ISR captures may lose CLK/DATA
        # transitions separated by only about 0.6...1.0 us, but the widely
        # separated CS boundaries remain useful for absolute boot timing.
        if 150.0 <= window["cs_low_us"] <= 250.0:
            result["command_windows"].append(window)
            if result["first_command_window"] is None:
                result["first_command_window"] = window

        if len(clocks) == 32 and result["first_valid_frame"] is None:
            value = _bits_to_value([bit for _, bit in clocks], False)
            result["first_valid_frame"] = {
                **window,
                "frame": value,
                "frame_hex": f"0x{value:08X}",
            }

    if result["first_valid_frame"] is not None:
        result["power_timing_verdict"] = "VALID 32-BIT FRAME FOUND"
    elif result["first_command_window"] is not None:
        result["power_timing_verdict"] = "COMMAND TIMING FOUND; EDGE DATA INCOMPLETE"
    else:
        result["power_timing_verdict"] = "NO COMPLETE 32-BIT FRAME IN CAPTURE"
    return result
