#!/usr/bin/env python3
"""
Calculate button voltage levels and SAR ADC values for the controller.

Circuit:
  Vcc ─┬─ BTN1 ─ R1 ─┬─ PIN ─ R_gnd ─ GND
       └─ BTN2 ─ R2 ─┘

When a button is open, that path is disconnected.
When a button is closed, it contributes to a voltage divider.
"""

VCC = 3.3  # Supply voltage
ADC_BITS = 4  # 4-bit SAR ADC
ADC_LEVELS = 2 ** ADC_BITS  # 16 levels (0-15)

def calc_voltage(r1_path: float | None, r2_path: float | None, r_gnd: float) -> float:
    """
    Calculate voltage at PIN given active resistor paths.
    
    r1_path: Resistance from Vcc through BTN1 path (None if open)
    r2_path: Resistance from Vcc through BTN2 path (None if open)
    r_gnd: Resistance to ground
    
    Returns voltage at PIN.
    """
    # Calculate equivalent resistance from Vcc to PIN
    paths = [r for r in [r1_path, r2_path] if r is not None]
    
    if not paths:
        # Both buttons open - PIN pulled to GND
        return 0.0
    
    # Parallel combination of all active paths
    r_top = 1.0 / sum(1.0 / r for r in paths)
    
    # Voltage divider
    return VCC * r_gnd / (r_top + r_gnd)


def voltage_to_sar(voltage: float) -> int:
    """
    Convert voltage to expected SAR ADC reading.
    
    SAR uses '>' comparison, so finds highest ref where input > ref.
    Each step is VCC/16.
    """
    step = VCC / ADC_LEVELS
    # SAR result is floor(voltage / step), capped at 15
    # But since we use '>' not '>=', we get n-1 for exact matches
    raw = voltage / step
    result = int(raw)
    if result > 0 and abs(raw - result) < 0.001:  # Exact match gives n-1
        result -= 1
    return min(result, ADC_LEVELS - 1)


def analyze_circuit(r1: float, r2: float, r_gnd: float, label: str = ""):
    """Analyze a button circuit configuration."""
    print(f"\n{'=' * 60}")
    if label:
        print(f"Configuration: {label}")
    print(f"R1 (BTN1 path) = {r1/1000:.1f}kΩ")
    print(f"R2 (BTN2 path) = {r2/1000:.1f}kΩ")
    print(f"R_gnd = {r_gnd/1000:.1f}kΩ")
    print(f"Vcc = {VCC}V, ADC = {ADC_BITS}-bit ({ADC_LEVELS} levels)")
    print(f"Step size = {VCC/ADC_LEVELS*1000:.1f}mV")
    print()
    
    states = [
        ("BTN1=0, BTN2=0", None, None),
        ("BTN1=1, BTN2=0", r1, None),
        ("BTN1=0, BTN2=1", None, r2),
        ("BTN1=1, BTN2=1", r1, r2),
    ]
    
    results = []
    print(f"{'State':<18} {'Voltage':>10} {'SAR Value':>10} {'btn1':>6} {'btn2':>6}")
    print("-" * 60)
    
    for name, r1_active, r2_active in states:
        v = calc_voltage(r1_active, r2_active, r_gnd)
        sar = voltage_to_sar(v)
        btn1 = 1 if r1_active else 0
        btn2 = 1 if r2_active else 0
        results.append((name, v, sar, btn1, btn2))
        print(f"{name:<18} {v:>9.3f}V {sar:>10} {btn1:>6} {btn2:>6}")
    
    # Check for good separation
    print()
    sar_values = [r[2] for r in results]
    sar_sorted = sorted(set(sar_values))
    
    if len(sar_sorted) < 4:
        print("⚠️  WARNING: Some states have identical SAR values!")
    else:
        gaps = [sar_sorted[i+1] - sar_sorted[i] for i in range(len(sar_sorted)-1)]
        min_gap = min(gaps)
        print(f"SAR value gaps: {gaps}")
        if min_gap < 2:
            print("⚠️  WARNING: Minimum gap < 2, may be unreliable")
        else:
            print(f"✓ Good separation (min gap = {min_gap})")
    
    # Generate C code thresholds
    print()
    print("Suggested C code thresholds:")
    
    # Sort by SAR value to determine thresholds
    by_sar = sorted(results, key=lambda x: x[2])
    
    thresholds = []
    for i in range(len(by_sar) - 1):
        curr_sar = by_sar[i][2]
        next_sar = by_sar[i+1][2]
        # Threshold is midpoint
        thresh = (curr_sar + next_sar) // 2 + 1
        thresholds.append(thresh)
    
    print("```c")
    print("// Button decode thresholds (auto-generated)")
    for i, (name, v, sar, btn1, btn2) in enumerate(by_sar):
        if i == 0:
            print(f"if (comp_val < {thresholds[0]}) {{")
        elif i < len(thresholds):
            print(f"}} else if (comp_val < {thresholds[i]}) {{")
        else:
            print("} else {")
        print(f"    new_btn1 = {btn1}; new_btn2 = {btn2};  // {name}, SAR≈{sar}")
    print("}")
    print("```")
    
    return results


def find_optimal_resistors():
    """Search for resistor values that give good separation."""
    print("\n" + "=" * 60)
    print("SEARCHING FOR OPTIMAL RESISTOR VALUES")
    print("=" * 60)
    
    # E12 series resistor values (common)
    e12 = [1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2]
    resistors = []
    for decade in [1000, 10000, 100000]:  # 1k, 10k, 100k
        for val in e12:
            resistors.append(val * decade)
    
    best = None
    best_score = -1
    
    for r1 in resistors:
        for r2 in resistors:
            if r2 <= r1:  # Avoid duplicates, ensure r2 > r1
                continue
            for r_gnd in resistors:
                # Calculate all 4 states
                v00 = calc_voltage(None, None, r_gnd)
                v10 = calc_voltage(r1, None, r_gnd)
                v01 = calc_voltage(None, r2, r_gnd)
                v11 = calc_voltage(r1, r2, r_gnd)
                
                sars = [voltage_to_sar(v) for v in [v00, v10, v01, v11]]
                
                # Score based on minimum gap between adjacent values
                sar_sorted = sorted(set(sars))
                if len(sar_sorted) < 4:
                    continue  # Some states collide
                
                gaps = [sar_sorted[i+1] - sar_sorted[i] for i in range(3)]
                min_gap = min(gaps)
                
                # Prefer solutions where values are evenly spaced
                score = min_gap * 10 + sum(gaps)
                
                # Prefer using common resistor values
                if r_gnd == 10000:
                    score += 5
                
                if score > best_score:
                    best_score = score
                    best = (r1, r2, r_gnd, sars, gaps)
    
    if best:
        r1, r2, r_gnd, sars, gaps = best
        print(f"\nBest found: R1={r1/1000:.1f}k, R2={r2/1000:.1f}k, R_gnd={r_gnd/1000:.1f}k")
        print(f"SAR values: {sars}, gaps: {gaps}")
        return r1, r2, r_gnd
    
    return None


def find_fast_binary_search_resistors():
    """
    Find resistor values optimized for 2-comparison binary search.
    
    With 4 states, we need log2(4) = 2 bits. Ideal target values:
    - State 0: SAR ≈ 0-3   (below 4)
    - State 1: SAR ≈ 4-7   (4 to below 8)
    - State 2: SAR ≈ 8-11  (8 to below 12)
    - State 3: SAR ≈ 12-15 (12 and above)
    
    This allows:
    - Compare at 8: splits into low pair vs high pair
    - Compare at 4 or 12: identifies exact state
    """
    print("\n" + "=" * 60)
    print("SEARCHING FOR 2-COMPARISON BINARY SEARCH VALUES")
    print("=" * 60)
    print("\nTarget: SAR values in ranges [0-3], [4-7], [8-11], [12-15]")
    print("This allows 2 comparisons instead of 4 for full SAR.\n")
    
    # Target SAR values - center of each quadrant for reliability
    targets = [1, 5, 9, 13]
    
    # E12 and E24 series
    e24 = [1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0,
           3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1]
    resistors = []
    for decade in [1000, 10000, 100000]:
        for val in e24:
            resistors.append(val * decade)
    
    candidates = []
    
    for r1 in resistors:
        for r2 in resistors:
            if r2 <= r1:
                continue
            for r_gnd in resistors:
                # Calculate all 4 states
                v00 = calc_voltage(None, None, r_gnd)
                v10 = calc_voltage(r1, None, r_gnd)
                v01 = calc_voltage(None, r2, r_gnd)
                v11 = calc_voltage(r1, r2, r_gnd)
                
                sars = [voltage_to_sar(v) for v in [v00, v10, v01, v11]]
                sar_sorted = sorted(sars)
                
                # Check if all 4 are distinct
                if len(set(sars)) < 4:
                    continue
                
                # Check if they fall in different quadrants
                quadrants = [s // 4 for s in sar_sorted]
                if quadrants != [0, 1, 2, 3]:
                    continue
                
                # Calculate minimum gap (for reliability)
                gaps = [sar_sorted[i+1] - sar_sorted[i] for i in range(3)]
                min_gap = min(gaps)
                
                # Score: prefer larger min_gap, then closeness to targets
                target_dist = sum((sar_sorted[i] - targets[i])**2 for i in range(4))
                
                # Prefer 10k ground resistor
                bonus = 1 if r_gnd == 10000 else 0
                
                candidates.append((min_gap, -target_dist, bonus, r1, r2, r_gnd, sars, sar_sorted, gaps))
    
    # Sort by min_gap descending, then target_dist ascending
    candidates.sort(reverse=True)
    
    if candidates:
        min_gap, neg_dist, bonus, r1, r2, r_gnd, sars, sar_sorted, gaps = candidates[0]
        print(f"Best found: R1={r1/1000:.2f}k, R2={r2/1000:.2f}k, R_gnd={r_gnd/1000:.2f}k")
        print(f"SAR values (state order): {sars}")
        print(f"SAR values (sorted): {sar_sorted}")
        print(f"Gaps between values: {gaps} (min={min_gap})")
        print(f"Quadrants: {[s // 4 for s in sar_sorted]} (target: [0,1,2,3])")
        
        # Show top 5 alternatives
        print("\nTop 5 alternatives (sorted by min gap):")
        for i, (mg, nd, b, r1_, r2_, rg_, sars_, sorted_, gaps_) in enumerate(candidates[:5]):
            print(f"  {i+1}. R1={r1_/1000:.1f}k R2={r2_/1000:.1f}k Rg={rg_/1000:.1f}k -> SAR {sorted_} gaps={gaps_}")
        
        return r1, r2, r_gnd
    else:
        print("No perfect solution found.")
        return None


def generate_fast_decode(results):
    """Generate C code for 2-comparison binary search decode."""
    print("\n" + "-" * 40)
    print("FAST 2-COMPARISON DECODE")
    print("-" * 40)
    
    # Sort by SAR value
    by_sar = sorted(results, key=lambda x: x[2])
    
    # Check quadrant placement
    quadrants = [r[2] // 4 for r in by_sar]
    
    if quadrants == [0, 1, 2, 3]:
        print("✓ Perfect quadrant placement!")
        print("\nOnly 2 comparisons needed:\n")
        
        print("```c")
        print("// Fast 2-comparison button decode")
        print("// Comparator reference levels: 8 and (4 or 12)")
        print("static void decode_buttons_fast(void) {")
        print("    uint8_t high_half, result;")
        print("    ")
        print("    // First comparison: ref=8 (splits into low/high pairs)")
        print("    comp_set_reference(8);")
        print("    high_half = comp_read();  // 1 if >= 8")
        print("    ")
        print("    // Second comparison: ref=4 or 12 depending on first result")
        print("    if (high_half) {")
        print("        comp_set_reference(12);")
        print("        result = comp_read();  // 1 if >= 12")
        print(f"        if (result) {{")
        print(f"            // Quadrant 3 (SAR 12-15): {by_sar[3][0]}")
        print(f"            new_btn1 = {by_sar[3][3]}; new_btn2 = {by_sar[3][4]};")
        print(f"        }} else {{")
        print(f"            // Quadrant 2 (SAR 8-11): {by_sar[2][0]}")
        print(f"            new_btn1 = {by_sar[2][3]}; new_btn2 = {by_sar[2][4]};")
        print(f"        }}")
        print("    } else {")
        print("        comp_set_reference(4);")
        print("        result = comp_read();  // 1 if >= 4")
        print(f"        if (result) {{")
        print(f"            // Quadrant 1 (SAR 4-7): {by_sar[1][0]}")
        print(f"            new_btn1 = {by_sar[1][3]}; new_btn2 = {by_sar[1][4]};")
        print(f"        }} else {{")
        print(f"            // Quadrant 0 (SAR 0-3): {by_sar[0][0]}")
        print(f"            new_btn1 = {by_sar[0][3]}; new_btn2 = {by_sar[0][4]};")
        print(f"        }}")
        print("    }")
        print("}")
        print("```")
        
        print("\nTiming improvement: 2 comparisons vs 4 = 50% faster button decode!")
    else:
        print(f"⚠️  Quadrant placement {quadrants} is not ideal [0,1,2,3]")
        print("Falling back to standard decode.")


if __name__ == "__main__":
    # Current circuit values
    print("CURRENT CIRCUIT ANALYSIS")
    analyze_circuit(
        r1=10000,    # 10k through BTN1
        r2=22000,    # 22k through BTN2  
        r_gnd=10000, # 10k to GND
        label="Current design"
    )
    
    # Find binary-search optimized values
    optimal = find_fast_binary_search_resistors()
    if optimal:
        print("\n\nOPTIMAL FOR FAST BINARY SEARCH")
        results = analyze_circuit(*optimal, label="Fast binary search")
        generate_fast_decode(results)
    
    # Also show what 2-comparison decode would look like with current values
    print("\n" + "=" * 60)
    print("CURRENT CIRCUIT - BINARY SEARCH ANALYSIS")
    r1, r2, r_gnd = 10000, 22000, 10000
    states = [
        ("BTN1=0, BTN2=0", None, None),
        ("BTN1=1, BTN2=0", r1, None),
        ("BTN1=0, BTN2=1", None, r2),
        ("BTN1=1, BTN2=1", r1, r2),
    ]
    results = []
    for name, r1_active, r2_active in states:
        v = calc_voltage(r1_active, r2_active, r_gnd)
        sar = voltage_to_sar(v)
        btn1 = 1 if r1_active else 0
        btn2 = 1 if r2_active else 0
        results.append((name, v, sar, btn1, btn2))
    
    by_sar = sorted(results, key=lambda x: x[2])
    quadrants = [r[2] // 4 for r in by_sar]
    print(f"Current SAR values: {[r[2] for r in by_sar]}")
    print(f"Current quadrants: {quadrants}")
    if quadrants == [0, 1, 2, 3]:
        print("✓ Current circuit already supports 2-comparison decode!")
        generate_fast_decode(results)
    else:
        print("✗ Current circuit needs 4-comparison SAR (values don't span quadrants)")

