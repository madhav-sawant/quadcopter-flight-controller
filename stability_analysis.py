"""
Quadcopter Control System Stability Analysis
=============================================
Analyzes the cascaded Angle + Rate PID loop for:
- Root Locus
- Bode Plot (Gain & Phase Margins)
- Step Response (Settling Time, Overshoot, Damping)

System Model:
- Plant: Quadcopter pitch axis (simplified as double integrator + motor dynamics)
- Inner Loop: Rate PID (gyro feedback)
- Outer Loop: Angle PID (attitude feedback, output = rate setpoint)

Hardware Constraints:
- Throttle limited to 1400us (high thrust-to-weight ratio)
- Hover at ~30% throttle (~1300us)
- Motor output range: 1100-1400us (only 300us of control authority)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# =============================================================================
# SYSTEM PARAMETERS
# =============================================================================

# Control Loop Frequency
LOOP_FREQ_HZ = 250  # 250 Hz
DT = 1.0 / LOOP_FREQ_HZ

# PID Gains (Updated Configuration - High Gain System)
# Rate PID (Inner Loop)
RATE_KP = 0.035
RATE_KI = 0.015
RATE_KD = 0.004

# Angle PID (Outer Loop)
ANGLE_KP = 3.50
ANGLE_KI = 0.30
ANGLE_KD = 0.00

# Motor/Aircraft Dynamics (Estimated for F450, 1400KV, 8045 props)
# High thrust-to-weight means HIGH torque gain

# Motor time constant (ESC + Motor inertia) - typically 20-50ms
TAU_MOTOR = 0.03  # 30ms

# Torque gain: deg/s^2 per unit PID output
# High for overpowered system. Estimated based on:
# - Hover at 30% throttle -> thrust margin is ~70%
# - Aggressive response observed in logs
TORQUE_GAIN = 800.0  # deg/s^2 per unit PID output (high for this system)

# Moment of inertia factor (scales torque to angular acceleration)
# Combined into TORQUE_GAIN for simplicity

# Output limits
RATE_OUTPUT_LIMIT = 150.0  # deg/s
PID_OUTPUT_LIMIT = 350.0   # PWM units (roughly)

# =============================================================================
# TRANSFER FUNCTION MODELS
# =============================================================================

def build_system():
    """
    Build the closed-loop transfer function for the cascaded controller.
    
    Plant Model (Pitch Axis):
    G_motor(s) = 1 / (TAU_MOTOR * s + 1)  -- Motor dynamics (1st order)
    G_inertia(s) = TORQUE_GAIN / s        -- Angular rate from torque (integrator)
    G_angle(s) = 1 / s                     -- Angle from rate (integrator)
    
    Full Plant: angle/pid_output = TORQUE_GAIN / (s^2 * (TAU_MOTOR*s + 1))
    """
    
    # --- Motor + Inertia (Rate Plant) ---
    # Rate output / PID output: G_rate = K / (s * (tau*s + 1))
    # This is a first-order lag + integrator
    num_rate_plant = [TORQUE_GAIN]
    den_rate_plant = [TAU_MOTOR, 1, 0]  # tau*s^2 + s (missing constant means integrator)
    # Correction: s*(tau*s + 1) = tau*s^2 + s
    den_rate_plant = np.convolve([TAU_MOTOR, 1], [1, 0])  # (tau*s + 1) * s
    G_rate_plant = signal.TransferFunction(num_rate_plant, den_rate_plant)
    
    # --- Rate PID Controller ---
    # C_rate(s) = Kp + Ki/s + Kd*s (idealized, no D-filter)
    # In practice, D is filtered. Let's add a simple filter: Kd*s / (Td*s + 1)
    TD_FILTER = 0.005  # D-term filter time constant (5ms)
    
    # PID = Kp + Ki/s + Kd*s/(Td*s+1)
    # Common denominator: s*(Td*s+1)
    # = [Kp*s*(Td*s+1) + Ki*(Td*s+1) + Kd*s^2] / [s*(Td*s+1)]
    # Numerator: Kp*Td*s^2 + Kp*s + Ki*Td*s + Ki + Kd*s^2
    #          = (Kp*Td + Kd)*s^2 + (Kp + Ki*Td)*s + Ki
    num_rate_pid = [(RATE_KP*TD_FILTER + RATE_KD), (RATE_KP + RATE_KI*TD_FILTER), RATE_KI]
    den_rate_pid = np.convolve([TD_FILTER, 1], [1, 0])  # s*(Td*s + 1) = Td*s^2 + s
    C_rate = signal.TransferFunction(num_rate_pid, den_rate_pid)
    
    # --- Closed Inner Loop (Rate) ---
    # L_rate = C_rate * G_rate_plant
    L_rate_num = np.convolve(num_rate_pid, num_rate_plant)
    L_rate_den = np.convolve(den_rate_pid, den_rate_plant)
    
    # T_rate = L_rate / (1 + L_rate)
    # For transfer function: T = num / (den + num) -- but only if open loop is num/den
    # Closed loop: T = G*C / (1 + G*C) 
    # Numerator = L_rate_num
    # Denominator = L_rate_den + L_rate_num (add coefficients properly)
    
    # Pad to same length
    max_len = max(len(L_rate_num), len(L_rate_den))
    L_rate_num_padded = np.pad(L_rate_num, (max_len - len(L_rate_num), 0))
    L_rate_den_padded = np.pad(L_rate_den, (max_len - len(L_rate_den), 0))
    
    T_rate_num = L_rate_num_padded
    T_rate_den = L_rate_den_padded + L_rate_num_padded
    T_rate = signal.TransferFunction(T_rate_num, T_rate_den)
    
    # --- Angle Plant ---
    # Angle = integral of Rate, so G_angle = 1/s
    G_angle_plant_num = [1]
    G_angle_plant_den = [1, 0]  # 1/s
    
    # Combined: Angle / Rate_Setpoint = T_rate * (1/s)
    G_angle_inner_num = np.convolve(T_rate_num, G_angle_plant_num)
    G_angle_inner_den = np.convolve(T_rate_den, G_angle_plant_den)
    
    # --- Angle PID Controller (Outer Loop) ---
    # Typically P + I only (Kd = 0)
    # C_angle(s) = Kp + Ki/s = (Kp*s + Ki) / s
    num_angle_pid = [ANGLE_KP, ANGLE_KI]
    den_angle_pid = [1, 0]  # s
    C_angle = signal.TransferFunction(num_angle_pid, den_angle_pid)
    
    # --- Open Loop (Angle) ---
    L_angle_num = np.convolve(num_angle_pid, G_angle_inner_num)
    L_angle_den = np.convolve(den_angle_pid, G_angle_inner_den)
    L_angle = signal.TransferFunction(L_angle_num, L_angle_den)
    
    # --- Closed Loop (Angle) ---
    max_len = max(len(L_angle_num), len(L_angle_den))
    L_angle_num_padded = np.pad(L_angle_num, (max_len - len(L_angle_num), 0))
    L_angle_den_padded = np.pad(L_angle_den, (max_len - len(L_angle_den), 0))
    
    T_angle_num = L_angle_num_padded
    T_angle_den = L_angle_den_padded + L_angle_num_padded
    T_angle = signal.TransferFunction(T_angle_num, T_angle_den)
    
    return {
        'rate_plant': G_rate_plant,
        'rate_pid': C_rate,
        'rate_closed': T_rate,
        'angle_open': L_angle,
        'angle_closed': T_angle,
    }


def analyze_stability(sys_dict):
    """Analyze stability metrics."""
    
    T_angle = sys_dict['angle_closed']
    L_angle = sys_dict['angle_open']
    
    # --- Poles & Zeros ---
    poles = np.roots(T_angle.den)
    zeros = np.roots(T_angle.num)
    
    print("=" * 60)
    print("CLOSED-LOOP POLE ANALYSIS")
    print("=" * 60)
    
    stable = True
    for i, p in enumerate(poles):
        real = np.real(p)
        imag = np.imag(p)
        if real >= 0:
            stable = False
            status = "UNSTABLE"
        else:
            status = "stable"
        
        # Calculate damping ratio for complex poles
        if np.abs(imag) > 1e-6:
            wn = np.abs(p)  # Natural frequency
            zeta = -real / wn  # Damping ratio
            print(f"  Pole {i+1}: {real:.4f} ± {abs(imag):.4f}j | ωn={wn:.2f} rad/s | ζ={zeta:.3f} | {status}")
        else:
            print(f"  Pole {i+1}: {real:.4f} (real) | {status}")
    
    print(f"\nSystem Stability: {'✓ STABLE' if stable else '✗ UNSTABLE'}")
    
    # --- Gain & Phase Margins ---
    print("\n" + "=" * 60)
    print("FREQUENCY DOMAIN ANALYSIS")
    print("=" * 60)
    
    # Bode plot data
    w = np.logspace(-1, 3, 1000)  # 0.1 to 1000 rad/s
    w, mag, phase = signal.bode(L_angle, w)
    
    # Find gain margin (phase = -180°, measure gain)
    phase_crossover_idx = np.where(np.diff(np.sign(phase + 180)))[0]
    if len(phase_crossover_idx) > 0:
        idx = phase_crossover_idx[0]
        gm_db = -mag[idx]  # Gain margin in dB
        gm_freq = w[idx]
        print(f"  Gain Margin: {gm_db:.2f} dB at {gm_freq:.2f} rad/s")
    else:
        gm_db = float('inf')
        print(f"  Gain Margin: ∞ (phase never crosses -180°)")
    
    # Find phase margin (gain = 0 dB, measure phase)
    gain_crossover_idx = np.where(np.diff(np.sign(mag)))[0]
    if len(gain_crossover_idx) > 0:
        idx = gain_crossover_idx[0]
        pm_deg = 180 + phase[idx]  # Phase margin
        pm_freq = w[idx]
        print(f"  Phase Margin: {pm_deg:.2f}° at {pm_freq:.2f} rad/s ({pm_freq/(2*np.pi):.2f} Hz)")
    else:
        pm_deg = float('inf')
        print(f"  Phase Margin: ∞ (gain never crosses 0 dB)")
    
    # --- Step Response ---
    print("\n" + "=" * 60)
    print("TIME DOMAIN ANALYSIS (Step Response)")
    print("=" * 60)
    
    t, y = signal.step(T_angle, T=np.linspace(0, 2, 2000))  # 2 seconds
    
    # Final value (should be 1.0 for unity gain)
    y_final = y[-1]
    
    # Rise time (10% to 90%)
    y_10 = 0.1 * y_final
    y_90 = 0.9 * y_final
    t_10_idx = np.where(y >= y_10)[0]
    t_90_idx = np.where(y >= y_90)[0]
    if len(t_10_idx) > 0 and len(t_90_idx) > 0:
        t_rise = t[t_90_idx[0]] - t[t_10_idx[0]]
    else:
        t_rise = float('inf')
    
    # Peak overshoot
    y_peak = np.max(y)
    overshoot_pct = (y_peak - y_final) / y_final * 100 if y_final > 0 else 0
    t_peak = t[np.argmax(y)]
    
    # Settling time (2% criterion)
    settling_band = 0.02 * y_final
    settled_idx = np.where(np.abs(y - y_final) > settling_band)[0]
    if len(settled_idx) > 0:
        t_settle = t[settled_idx[-1]]
    else:
        t_settle = 0
    
    # Dominant poles -> damping ratio estimate
    complex_poles = poles[np.abs(np.imag(poles)) > 0.01]
    if len(complex_poles) > 0:
        # Find dominant (slowest) complex pair
        dominant = complex_poles[np.argmax(np.real(complex_poles))]
        wn = np.abs(dominant)
        zeta = -np.real(dominant) / wn
    else:
        wn = 0
        zeta = 1.0  # Overdamped
    
    print(f"  Rise Time (10-90%):   {t_rise*1000:.1f} ms")
    print(f"  Peak Time:            {t_peak*1000:.1f} ms")
    print(f"  Peak Overshoot:       {overshoot_pct:.1f}%")
    print(f"  Settling Time (2%):   {t_settle*1000:.1f} ms")
    print(f"  Damping Ratio (ζ):    {zeta:.3f}")
    print(f"  Natural Freq (ωn):    {wn:.2f} rad/s ({wn/(2*np.pi):.2f} Hz)")
    
    # --- Motor Saturation Check ---
    print("\n" + "=" * 60)
    print("MOTOR SATURATION ANALYSIS")
    print("=" * 60)
    
    # For a 10° step command, estimate peak PID output
    step_angle = 10.0  # degrees
    
    # Peak rate setpoint (from angle P-term at max error)
    peak_rate_sp = ANGLE_KP * step_angle  # deg/s
    print(f"  10° Step Command:")
    print(f"    Peak Rate Setpoint: {peak_rate_sp:.1f} deg/s (Limit: {RATE_OUTPUT_LIMIT} deg/s)")
    
    # If rate setpoint exceeds limit, it will be clamped
    clamped_rate = min(peak_rate_sp, RATE_OUTPUT_LIMIT)
    
    # Peak PID output (from rate P-term at max rate error)
    peak_pid = RATE_KP * clamped_rate  # Simplified (ignores D-spike)
    print(f"    Peak Rate PID Output: ~{peak_pid:.1f} (before D-spike)")
    
    # Motor authority check
    # Throttle = 1300us (hover), Range = 1100-1400us = ±150us from hover
    motor_authority = 150  # us
    print(f"    Motor Authority (1300±150us): ±{motor_authority} us")
    print(f"    Estimated Saturation: {'YES' if peak_pid > motor_authority else 'NO'}")
    
    return {
        'stable': stable,
        'gm_db': gm_db,
        'pm_deg': pm_deg,
        't_rise': t_rise,
        't_settle': t_settle,
        'overshoot': overshoot_pct,
        'zeta': zeta,
        'wn': wn,
        't': t,
        'y': y,
        'w': w,
        'mag': mag,
        'phase': phase,
    }


def plot_results(analysis, sys_dict):
    """Generate analysis plots."""
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # --- Step Response ---
    ax = axes[0, 0]
    ax.plot(analysis['t'] * 1000, analysis['y'], 'b-', linewidth=2)
    ax.axhline(y=1.0, color='k', linestyle='--', alpha=0.5, label='Setpoint')
    ax.axhline(y=1.02, color='g', linestyle=':', alpha=0.5)
    ax.axhline(y=0.98, color='g', linestyle=':', alpha=0.5, label='±2% Band')
    ax.axvline(x=analysis['t_settle']*1000, color='r', linestyle='--', alpha=0.7, 
               label=f"Settle: {analysis['t_settle']*1000:.0f}ms")
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Angle (normalized)')
    ax.set_title('Closed-Loop Step Response')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 1000])
    
    # --- Bode Plot ---
    ax1 = axes[0, 1]
    ax1.semilogx(analysis['w'], analysis['mag'], 'b-', linewidth=2)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax1.set_ylabel('Magnitude (dB)')
    ax1.set_title('Open-Loop Bode Plot')
    ax1.grid(True, alpha=0.3, which='both')
    
    ax2 = ax1.twinx()
    ax2.semilogx(analysis['w'], analysis['phase'], 'r-', linewidth=2, alpha=0.7)
    ax2.axhline(y=-180, color='r', linestyle='--', alpha=0.5)
    ax2.set_ylabel('Phase (deg)', color='r')
    
    # --- Root Locus (Pole-Zero Map) ---
    ax = axes[1, 0]
    T_angle = sys_dict['angle_closed']
    poles = np.roots(T_angle.den)
    zeros = np.roots(T_angle.num)
    
    ax.scatter(np.real(poles), np.imag(poles), marker='x', s=100, c='red', label='Poles')
    if len(zeros) > 0:
        ax.scatter(np.real(zeros), np.imag(zeros), marker='o', s=100, 
                   facecolors='none', edgecolors='blue', label='Zeros')
    ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)
    ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
    ax.set_xlabel('Real')
    ax.set_ylabel('Imaginary')
    ax.set_title('Pole-Zero Map (Closed-Loop)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    
    # --- Summary Text ---
    ax = axes[1, 1]
    ax.axis('off')
    
    summary = f"""
    ╔══════════════════════════════════════════════════╗
    ║          STABILITY ANALYSIS SUMMARY              ║
    ╠══════════════════════════════════════════════════╣
    ║  System Status:    {'✓ STABLE' if analysis['stable'] else '✗ UNSTABLE':>28} ║
    ║                                                  ║
    ║  Gain Margin:      {analysis['gm_db']:>24.1f} dB  ║
    ║  Phase Margin:     {analysis['pm_deg']:>24.1f} °   ║
    ║                                                  ║
    ║  Rise Time:        {analysis['t_rise']*1000:>24.1f} ms  ║
    ║  Settling Time:    {analysis['t_settle']*1000:>24.1f} ms  ║
    ║  Peak Overshoot:   {analysis['overshoot']:>24.1f} %   ║
    ║                                                  ║
    ║  Damping Ratio ζ:  {analysis['zeta']:>24.3f}     ║
    ║  Natural Freq ωn:  {analysis['wn']:>21.2f} rad/s  ║
    ╚══════════════════════════════════════════════════╝
    
    PID Configuration:
    ────────────────────
    Rate Loop:   Kp={RATE_KP}, Ki={RATE_KI}, Kd={RATE_KD}
    Angle Loop:  Kp={ANGLE_KP}, Ki={ANGLE_KI}, Kd={ANGLE_KD}
    
    Hardware Constraints:
    ────────────────────
    Throttle Limit: 1400us (aggressive for high T/W)
    Motor Authority: ~±150us from hover
    """
    
    ax.text(0.05, 0.95, summary, transform=ax.transAxes, fontsize=10,
            verticalalignment='top', fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    plt.tight_layout()
    plt.savefig('/home/maddy/Videos/Firmware/stability_analysis.png', dpi=150)
    print("\nPlot saved to: /home/maddy/Videos/Firmware/stability_analysis.png")
    plt.close()


if __name__ == "__main__":
    print("Building control system model...")
    sys_dict = build_system()
    
    print("\nAnalyzing stability...\n")
    analysis = analyze_stability(sys_dict)
    
    print("\nGenerating plots...")
    plot_results(analysis, sys_dict)
    
    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE")
    print("=" * 60)
