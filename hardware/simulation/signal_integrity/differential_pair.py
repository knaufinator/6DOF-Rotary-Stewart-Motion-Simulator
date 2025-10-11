"""
Differential Pair Impedance Calculator

Calculates the characteristic impedance of differential pairs for PCB design.
Used to validate that 0.25mm width / 0.25mm gap achieves 120Ω for RS-422.

Based on microstrip differential pair equations.
"""

import numpy as np
import argparse


def calculate_single_ended_impedance(w, h, t, er):
    """
    Calculate single-ended microstrip impedance.
    
    Args:
        w: Trace width (mm)
        h: Dielectric height (mm)
        t: Trace thickness (mm, typically 0.035mm for 1oz copper)
        er: Relative permittivity (FR-4 ≈ 4.5)
    
    Returns:
        Z0: Characteristic impedance (Ω)
    """
    # Effective width accounting for trace thickness
    w_eff = w + (t / np.pi) * (1 + np.log(2 * h / t))
    
    # Effective permittivity
    er_eff = (er + 1) / 2 + ((er - 1) / 2) * (1 / np.sqrt(1 + 12 * h / w_eff))
    
    # Characteristic impedance (Wheeler's approximation)
    if w_eff / h < 1:
        Z0 = (60 / np.sqrt(er_eff)) * np.log(8 * h / w_eff + w_eff / (4 * h))
    else:
        Z0 = (120 * np.pi / np.sqrt(er_eff)) / (w_eff / h + 1.393 + 0.667 * np.log(w_eff / h + 1.444))
    
    return Z0


def calculate_differential_impedance(w, s, h, t, er):
    """
    Calculate differential pair impedance.
    
    Args:
        w: Trace width (mm)
        s: Trace-to-trace spacing/gap (mm)
        h: Dielectric height (mm)
        t: Trace thickness (mm)
        er: Relative permittivity
    
    Returns:
        dict: Contains Z_diff, Z_odd, Z_even, Z_common
    """
    # Single-ended impedance
    Z0 = calculate_single_ended_impedance(w, h, t, er)
    
    # Coupling factor (empirical)
    # For microstrip, coupling decreases with spacing
    k = np.exp(-2.9 * s / h)
    
    # Odd-mode impedance (differential)
    Z_odd = Z0 * np.sqrt((1 - k) / (1 + k))
    
    # Even-mode impedance (common-mode)
    Z_even = Z0 * np.sqrt((1 + k) / (1 - k))
    
    # Differential impedance
    Z_diff = 2 * Z_odd
    
    # Common-mode impedance
    Z_common = Z_even / 2
    
    return {
        'Z_diff': Z_diff,
        'Z_odd': Z_odd,
        'Z_even': Z_even,
        'Z_common': Z_common,
        'Z0_single': Z0,
        'coupling_factor': k
    }


def main():
    parser = argparse.ArgumentParser(
        description='Calculate differential pair impedance for PCB design'
    )
    parser.add_argument('--width', type=float, default=0.25,
                       help='Trace width in mm (default: 0.25)')
    parser.add_argument('--gap', type=float, default=0.25,
                       help='Trace-to-trace gap in mm (default: 0.25)')
    parser.add_argument('--height', type=float, default=0.2,
                       help='Dielectric height in mm (default: 0.2)')
    parser.add_argument('--thickness', type=float, default=0.035,
                       help='Copper thickness in mm (default: 0.035 for 1oz)')
    parser.add_argument('--er', type=float, default=4.5,
                       help='Relative permittivity (default: 4.5 for FR-4)')
    parser.add_argument('--target', type=float, default=120.0,
                       help='Target differential impedance (default: 120Ω for RS-422)')
    
    args = parser.parse_args()
    
    # Calculate impedance
    result = calculate_differential_impedance(
        w=args.width,
        s=args.gap,
        h=args.height,
        t=args.thickness,
        er=args.er
    )
    
    # Display results
    print("=" * 70)
    print("DIFFERENTIAL PAIR IMPEDANCE CALCULATOR")
    print("=" * 70)
    print()
    print("PCB Parameters:")
    print(f"  Trace Width (w):        {args.width:.3f} mm")
    print(f"  Trace Gap (s):          {args.gap:.3f} mm")
    print(f"  Dielectric Height (h):  {args.height:.3f} mm")
    print(f"  Copper Thickness (t):   {args.thickness:.3f} mm ({args.thickness/0.035:.1f}oz)")
    print(f"  Relative Permittivity:  {args.er:.2f} (FR-4)")
    print()
    print("Calculated Impedances:")
    print(f"  Single-Ended Z₀:        {result['Z0_single']:.2f} Ω")
    print(f"  Odd-Mode Z_odd:         {result['Z_odd']:.2f} Ω")
    print(f"  Even-Mode Z_even:       {result['Z_even']:.2f} Ω")
    print(f"  Differential Z_diff:    {result['Z_diff']:.2f} Ω  ⭐")
    print(f"  Common-Mode Z_common:   {result['Z_common']:.2f} Ω")
    print(f"  Coupling Factor (k):    {result['coupling_factor']:.4f}")
    print()
    
    # Validate against target
    tolerance = 0.10  # ±10%
    z_diff = result['Z_diff']
    z_min = args.target * (1 - tolerance)
    z_max = args.target * (1 + tolerance)
    
    print("Specification Check:")
    print(f"  Target Impedance:       {args.target:.1f} Ω ±{tolerance*100:.0f}%")
    print(f"  Acceptable Range:       {z_min:.1f} Ω to {z_max:.1f} Ω")
    print(f"  Calculated Impedance:   {z_diff:.2f} Ω")
    
    if z_min <= z_diff <= z_max:
        print(f"  Status:                 ✅ PASS (within ±{tolerance*100:.0f}%)")
        error_pct = abs(z_diff - args.target) / args.target * 100
        print(f"  Error:                  {error_pct:.2f}%")
        return 0
    else:
        print(f"  Status:                 ❌ FAIL (outside tolerance)")
        error_pct = abs(z_diff - args.target) / args.target * 100
        print(f"  Error:                  {error_pct:.2f}%")
        
        # Suggest corrections
        print()
        print("Suggestions:")
        if z_diff < z_min:
            print("  - Increase trace spacing (gap)")
            print("  - Decrease trace width")
            print("  - Increase dielectric height")
        else:
            print("  - Decrease trace spacing (gap)")
            print("  - Increase trace width")
            print("  - Decrease dielectric height")
        
        return 1
    
    print("=" * 70)


if __name__ == '__main__':
    import sys
    sys.exit(main())
