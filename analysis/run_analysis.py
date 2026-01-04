"""
Quadcopter Control System Stability Analysis - Comprehensive Report
===================================================================
Generates detailed visualizations including:
- S-Plane (Pole-Zero Map)
- Stability Analysis
- Damping Factor Analysis
- Transient Response
- Settling Time Analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import matplotlib.patches as mpatches

# =============================================================================
# SYSTEM PARAMETERS (FROM YOUR ACTUAL FIRMWARE)
# =============================================================================

# Control Loop Frequency
LOOP_FREQ_HZ = 250  # 250 Hz
DT = 1.0 / LOOP_FREQ_HZ

# Rate PID (Inner Loop) - From config.c
RATE_KP = 0.40
RATE_KI = 0.20
RATE_KD = 0.03

# Angle PID (Outer Loop) - From stability_analysis.py
ANGLE_KP = 3.50
ANGLE_KI = 0.30
ANGLE_KD = 0.00

# Motor/Aircraft Dynamics (F450, 1400KV, 8045 props)
TAU_MOTOR = 0.03  # 30ms motor time constant
TORQUE_GAIN = 800.0  # deg/s^2 per unit PID output

# D-term filter (from pid.c)
D_TERM_LPF_ALPHA = 0.42
TD_FILTER = 0.005  # D-term filter time constant (5ms)

# Output limits
RATE_OUTPUT_LIMIT = 110.0  # From config.c
PID_OUTPUT_LIMIT = 350.0

# =============================================================================
# BUILD TRANSFER FUNCTION MODEL
# =============================================================================

def build_system():
    """Build the closed-loop transfer function for the cascaded controller."""
    
    # --- Motor + Inertia (Rate Plant) ---
    num_rate_plant = [TORQUE_GAIN]
    den_rate_plant = np.convolve([TAU_MOTOR, 1], [1, 0])  # (tau*s + 1) * s
    G_rate_plant = signal.TransferFunction(num_rate_plant, den_rate_plant)
    
    # --- Rate PID Controller ---
    num_rate_pid = [(RATE_KP*TD_FILTER + RATE_KD), (RATE_KP + RATE_KI*TD_FILTER), RATE_KI]
    den_rate_pid = np.convolve([TD_FILTER, 1], [1, 0])
    C_rate = signal.TransferFunction(num_rate_pid, den_rate_pid)
    
    # --- Closed Inner Loop (Rate) ---
    L_rate_num = np.convolve(num_rate_pid, num_rate_plant)
    L_rate_den = np.convolve(den_rate_pid, den_rate_plant)
    
    max_len = max(len(L_rate_num), len(L_rate_den))
    L_rate_num_padded = np.pad(L_rate_num, (max_len - len(L_rate_num), 0))
    L_rate_den_padded = np.pad(L_rate_den, (max_len - len(L_rate_den), 0))
    
    T_rate_num = L_rate_num_padded
    T_rate_den = L_rate_den_padded + L_rate_num_padded
    T_rate = signal.TransferFunction(T_rate_num, T_rate_den)
    
    # --- Angle Plant (1/s) ---
    G_angle_plant_num = [1]
    G_angle_plant_den = [1, 0]
    
    # Combined: Angle / Rate_Setpoint = T_rate * (1/s)
    G_angle_inner_num = np.convolve(T_rate_num, G_angle_plant_num)
    G_angle_inner_den = np.convolve(T_rate_den, G_angle_plant_den)
    
    # --- Angle PID Controller (Outer Loop) ---
    num_angle_pid = [ANGLE_KP, ANGLE_KI]
    den_angle_pid = [1, 0]
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

# =============================================================================
# FIGURE 1: S-PLANE (POLE-ZERO MAP)
# =============================================================================

def plot_splane(sys_dict, save_path):
    """Generate detailed S-Plane plot with stability regions."""
    
    T_angle = sys_dict['angle_closed']
    poles = np.roots(T_angle.den)
    zeros = np.roots(T_angle.num)
    
    fig, ax = plt.subplots(figsize=(12, 10))
    
    # Draw stability regions
    # Left half plane (stable region)
    ax.axvspan(-200, 0, alpha=0.1, color='green', label='Stable Region (LHP)')
    # Right half plane (unstable region)
    ax.axvspan(0, 50, alpha=0.1, color='red', label='Unstable Region (RHP)')
    
    # Draw imaginary axis (stability boundary)
    ax.axvline(x=0, color='black', linewidth=2, linestyle='-', label='Imaginary Axis (jω)')
    ax.axhline(y=0, color='black', linewidth=1, linestyle='-', alpha=0.5)
    
    # Draw damping ratio lines
    zetas = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]
    for zeta in zetas:
        theta = np.arccos(zeta)
        line_length = 180
        x_end = -line_length * np.cos(theta)
        y_end = line_length * np.sin(theta)
        ax.plot([0, x_end], [0, y_end], 'gray', linestyle='--', alpha=0.4, linewidth=0.8)
        ax.plot([0, x_end], [0, -y_end], 'gray', linestyle='--', alpha=0.4, linewidth=0.8)
        ax.text(x_end * 0.6, y_end * 0.6 + 5, f'ζ={zeta}', fontsize=8, alpha=0.6, rotation=-np.degrees(theta))
    
    # Plot poles
    for i, p in enumerate(poles):
        real, imag = np.real(p), np.imag(p)
        color = 'green' if real < 0 else 'red'
        ax.scatter(real, imag, marker='x', s=200, c=color, linewidths=3, zorder=5)
        
        # Calculate damping ratio and natural frequency
        if np.abs(imag) > 1e-6:
            wn = np.abs(p)
            zeta = -real / wn
            label = f'Pole {i+1}\nσ={real:.2f}\nωd={abs(imag):.1f}\nζ={zeta:.3f}'
        else:
            label = f'Pole {i+1}\nσ={real:.4f}'
        
        ax.annotate(label, (real, imag), xytext=(10, 10), 
                    textcoords='offset points', fontsize=8,
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
    
    # Plot zeros
    if len(zeros) > 0:
        for z in zeros:
            ax.scatter(np.real(z), np.imag(z), marker='o', s=150, 
                       facecolors='none', edgecolors='blue', linewidths=2, zorder=5)
    
    ax.set_xlabel('Real Axis (σ) [rad/s]', fontsize=12)
    ax.set_ylabel('Imaginary Axis (jω) [rad/s]', fontsize=12)
    ax.set_title('S-PLANE: Pole-Zero Map\nQuadcopter Cascaded Control System', fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='upper right')
    ax.set_xlim([-180, 50])
    ax.set_ylim([-150, 150])
    
    # Add stability verdict
    stable = all(np.real(p) < 0 for p in poles)
    verdict = "✓ SYSTEM IS STABLE\nAll poles in Left Half Plane" if stable else "✗ SYSTEM IS UNSTABLE\nPoles in Right Half Plane!"
    color = 'green' if stable else 'red'
    ax.text(0.02, 0.98, verdict, transform=ax.transAxes, fontsize=14, fontweight='bold',
            verticalalignment='top', color=color,
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {save_path}")

# =============================================================================
# FIGURE 2: STABILITY CLASSIFICATION
# =============================================================================

def plot_stability_classification(sys_dict, save_path):
    """Determine and visualize stability type: Stable, Unstable, or Marginally Stable."""
    
    T_angle = sys_dict['angle_closed']
    poles = np.roots(T_angle.den)
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # Classify each pole
    stable_poles = []
    unstable_poles = []
    marginal_poles = []
    
    for p in poles:
        real = np.real(p)
        if real < -1e-6:
            stable_poles.append(p)
        elif real > 1e-6:
            unstable_poles.append(p)
        else:
            marginal_poles.append(p)
    
    # Determine overall stability
    if len(unstable_poles) > 0:
        stability_type = "UNSTABLE"
        stability_color = "red"
        stability_desc = "Poles in Right Half Plane\nSystem response grows unbounded"
    elif len(marginal_poles) > 0:
        stability_type = "MARGINALLY STABLE"
        stability_color = "orange"
        stability_desc = "Poles on Imaginary Axis\nSustained oscillations possible"
    else:
        stability_type = "ASYMPTOTICALLY STABLE"
        stability_color = "green"
        stability_desc = "All poles in Left Half Plane\nSystem returns to equilibrium"
    
    # --- Panel 1: Classification ---
    ax = axes[0]
    ax.axis('off')
    ax.text(0.5, 0.8, "STABILITY CLASSIFICATION", fontsize=16, fontweight='bold',
            ha='center', transform=ax.transAxes)
    ax.text(0.5, 0.55, stability_type, fontsize=24, fontweight='bold',
            ha='center', color=stability_color, transform=ax.transAxes)
    ax.text(0.5, 0.35, stability_desc, fontsize=12, ha='center', transform=ax.transAxes)
    ax.text(0.5, 0.15, f"Stable poles: {len(stable_poles)}\nMarginal poles: {len(marginal_poles)}\nUnstable poles: {len(unstable_poles)}", 
            fontsize=11, ha='center', transform=ax.transAxes)
    
    # --- Panel 2: Pole Location Summary ---
    ax = axes[1]
    categories = ['Stable\n(Re < 0)', 'Marginal\n(Re = 0)', 'Unstable\n(Re > 0)']
    counts = [len(stable_poles), len(marginal_poles), len(unstable_poles)]
    colors = ['green', 'orange', 'red']
    bars = ax.bar(categories, counts, color=colors, edgecolor='black', linewidth=2)
    ax.set_ylabel('Number of Poles', fontsize=12)
    ax.set_title('Pole Distribution by Stability Region', fontsize=13, fontweight='bold')
    ax.set_ylim(0, max(counts) + 1)
    for bar, count in zip(bars, counts):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1, 
                str(int(count)), ha='center', fontsize=14, fontweight='bold')
    
    # --- Panel 3: Pole Values Table ---
    ax = axes[2]
    ax.axis('off')
    ax.text(0.5, 0.95, "CLOSED-LOOP POLES", fontsize=14, fontweight='bold',
            ha='center', transform=ax.transAxes)
    
    table_text = "Pole | Real | Imag | ωn | ζ | Status\n"
    table_text += "-" * 55 + "\n"
    
    for i, p in enumerate(poles):
        real = np.real(p)
        imag = np.imag(p)
        if np.abs(imag) > 1e-6:
            wn = np.abs(p)
            zeta = -real / wn
            status = "Stable" if real < 0 else "Unstable" if real > 0 else "Marginal"
            table_text += f"  {i+1}  | {real:7.2f} | {imag:7.2f}j | {wn:5.1f} | {zeta:.3f} | {status}\n"
        else:
            status = "Stable" if real < 0 else "Unstable" if real > 0 else "Marginal"
            table_text += f"  {i+1}  | {real:7.4f} |    0     |   -   |   -   | {status}\n"
    
    ax.text(0.05, 0.85, table_text, transform=ax.transAxes, fontsize=9,
            fontfamily='monospace', verticalalignment='top')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {save_path}")
    
    return stability_type

# =============================================================================
# FIGURE 3: DAMPING FACTOR ANALYSIS
# =============================================================================

def plot_damping_analysis(sys_dict, save_path):
    """Analyze and visualize damping characteristics."""
    
    T_angle = sys_dict['angle_closed']
    poles = np.roots(T_angle.den)
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Extract complex pole pairs for damping analysis
    complex_poles = []
    for p in poles:
        if np.abs(np.imag(p)) > 1e-6 and np.imag(p) >= 0:  # Upper half only (conjugate pairs)
            complex_poles.append(p)
    
    # --- Panel 1: Damping Ratio Visualization ---
    ax = axes[0, 0]
    
    # Draw reference damping circles
    theta = np.linspace(0, np.pi, 100)
    for r in [50, 100, 150]:
        ax.plot(-r * np.cos(theta), r * np.sin(theta), 'gray', alpha=0.3)
    
    # Draw damping lines
    for zeta in [0.2, 0.4, 0.6, 0.8, 1.0]:
        if zeta < 1:
            line_theta = np.arccos(zeta)
            ax.plot([0, -180*np.cos(line_theta)], [0, 180*np.sin(line_theta)], 
                    'b--', alpha=0.4, linewidth=1)
            ax.text(-90*np.cos(line_theta), 90*np.sin(line_theta) + 5, 
                    f'ζ={zeta}', fontsize=9, color='blue', alpha=0.7)
    
    # Plot poles with damping color coding
    for p in poles:
        real, imag = np.real(p), np.imag(p)
        if np.abs(imag) > 1e-6:
            wn = np.abs(p)
            zeta = -real / wn
            # Color based on damping
            if zeta < 0.4:
                color = 'red'  # Underdamped (oscillatory)
            elif zeta < 0.7:
                color = 'orange'  # Moderately damped
            elif zeta < 1.0:
                color = 'green'  # Well damped
            else:
                color = 'blue'  # Overdamped
            ax.scatter(real, imag, marker='x', s=200, c=color, linewidths=3)
    
    ax.axvline(x=0, color='black', linewidth=1)
    ax.axhline(y=0, color='black', linewidth=1)
    ax.set_xlabel('Real (σ)')
    ax.set_ylabel('Imaginary (jω)')
    ax.set_title('Pole Locations with Damping Lines', fontsize=12, fontweight='bold')
    ax.set_xlim([-180, 20])
    ax.set_ylim([-150, 150])
    ax.grid(True, alpha=0.3)
    
    # Legend
    legend_elements = [
        plt.Line2D([0], [0], marker='x', color='w', markeredgecolor='red', markersize=10, label='Underdamped (ζ<0.4)'),
        plt.Line2D([0], [0], marker='x', color='w', markeredgecolor='orange', markersize=10, label='Moderate (0.4≤ζ<0.7)'),
        plt.Line2D([0], [0], marker='x', color='w', markeredgecolor='green', markersize=10, label='Well damped (0.7≤ζ<1)'),
        plt.Line2D([0], [0], marker='x', color='w', markeredgecolor='blue', markersize=10, label='Overdamped (ζ≥1)'),
    ]
    ax.legend(handles=legend_elements, loc='upper right', fontsize=9)
    
    # --- Panel 2: Damping Ratios Bar Chart ---
    ax = axes[0, 1]
    damping_data = []
    labels = []
    colors = []
    
    for i, p in enumerate(poles):
        if np.abs(np.imag(p)) > 1e-6:
            wn = np.abs(p)
            zeta = -np.real(p) / wn
            damping_data.append(zeta)
            labels.append(f'Pole {i+1}\n(ωn={wn:.1f})')
            if zeta < 0.4:
                colors.append('red')
            elif zeta < 0.7:
                colors.append('orange')
            elif zeta < 1.0:
                colors.append('green')
            else:
                colors.append('blue')
    
    if len(damping_data) > 0:
        bars = ax.bar(labels, damping_data, color=colors, edgecolor='black')
        ax.axhline(y=0.707, color='green', linestyle='--', linewidth=2, label='Optimal ζ=0.707')
        ax.axhline(y=1.0, color='blue', linestyle=':', linewidth=2, label='Critical ζ=1.0')
        ax.set_ylabel('Damping Ratio (ζ)', fontsize=11)
        ax.set_title('Damping Ratios of Complex Poles', fontsize=12, fontweight='bold')
        ax.legend()
        ax.set_ylim(0, max(damping_data) * 1.2)
        for bar, val in zip(bars, damping_data):
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.02, 
                    f'{val:.3f}', ha='center', fontsize=10, fontweight='bold')
    
    # --- Panel 3: Damping Interpretation ---
    ax = axes[1, 0]
    ax.axis('off')
    
    # Get dominant pole (slowest, determines system response)
    if len(complex_poles) > 0:
        dominant = max(complex_poles, key=lambda p: np.real(p))  # Least negative real part
        dom_wn = np.abs(dominant)
        dom_zeta = -np.real(dominant) / dom_wn
    else:
        dom_wn = 0
        dom_zeta = 1.0
    
    # Interpretation
    if dom_zeta < 0.4:
        interp = "UNDERDAMPED (Oscillatory)\n• Expect significant overshoot\n• Multiple oscillations before settling\n• May feel 'bouncy' or unstable"
        interp_color = 'red'
    elif dom_zeta < 0.7:
        interp = "MODERATELY DAMPED\n• Some overshoot expected (5-15%)\n• Quick response but with ringing\n• Generally acceptable for flight"
        interp_color = 'orange'
    elif dom_zeta < 1.0:
        interp = "WELL DAMPED (Near Optimal)\n• Minimal overshoot (<5%)\n• Fast settling with little ringing\n• Excellent for stable flight"
        interp_color = 'green'
    else:
        interp = "OVERDAMPED\n• No overshoot\n• Slow response\n• May feel 'sluggish'"
        interp_color = 'blue'
    
    ax.text(0.5, 0.9, "DOMINANT POLE ANALYSIS", fontsize=14, fontweight='bold',
            ha='center', transform=ax.transAxes)
    ax.text(0.5, 0.75, f"Dominant Damping Ratio: ζ = {dom_zeta:.3f}", fontsize=16,
            ha='center', color=interp_color, fontweight='bold', transform=ax.transAxes)
    ax.text(0.5, 0.6, f"Natural Frequency: ωn = {dom_wn:.2f} rad/s ({dom_wn/(2*np.pi):.2f} Hz)", 
            fontsize=12, ha='center', transform=ax.transAxes)
    ax.text(0.5, 0.35, interp, fontsize=12, ha='center', transform=ax.transAxes,
            bbox=dict(boxstyle='round', facecolor=interp_color, alpha=0.2))
    
    # --- Panel 4: Damping Reference Chart ---
    ax = axes[1, 1]
    zetas = np.linspace(0, 1.5, 100)
    overshoots = []
    for z in zetas:
        if z < 1:
            os = 100 * np.exp(-np.pi * z / np.sqrt(1 - z**2))
        else:
            os = 0
        overshoots.append(os)
    
    ax.plot(zetas, overshoots, 'b-', linewidth=2)
    ax.axvline(x=dom_zeta, color='red', linestyle='--', linewidth=2, 
               label=f'Your system: ζ={dom_zeta:.3f}')
    ax.axvline(x=0.707, color='green', linestyle=':', linewidth=2, label='Optimal: ζ=0.707')
    ax.set_xlabel('Damping Ratio (ζ)', fontsize=11)
    ax.set_ylabel('Overshoot (%)', fontsize=11)
    ax.set_title('Overshoot vs Damping Ratio', fontsize=12, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 1.5)
    ax.set_ylim(0, 100)
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {save_path}")
    
    return dom_zeta, dom_wn

# =============================================================================
# FIGURE 4: TRANSIENT RESPONSE ANALYSIS
# =============================================================================

def plot_transient_response(sys_dict, save_path):
    """Analyze and visualize transient response characteristics."""
    
    T_angle = sys_dict['angle_closed']
    t, y = signal.step(T_angle, T=np.linspace(0, 2, 2000))
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Calculate metrics
    y_final = y[-1]
    y_peak = np.max(y)
    t_peak = t[np.argmax(y)]
    overshoot_pct = (y_peak - y_final) / y_final * 100 if y_final > 0 else 0
    
    # Rise time (10% to 90%)
    y_10 = 0.1 * y_final
    y_90 = 0.9 * y_final
    t_10_idx = np.where(y >= y_10)[0]
    t_90_idx = np.where(y >= y_90)[0]
    if len(t_10_idx) > 0 and len(t_90_idx) > 0:
        t_rise = t[t_90_idx[0]] - t[t_10_idx[0]]
        t_10 = t[t_10_idx[0]]
        t_90 = t[t_90_idx[0]]
    else:
        t_rise = float('inf')
        t_10 = 0
        t_90 = 0
    
    # Settling time (2% criterion)
    settling_band = 0.02 * y_final
    settled_idx = np.where(np.abs(y - y_final) > settling_band)[0]
    if len(settled_idx) > 0:
        t_settle = t[settled_idx[-1]]
    else:
        t_settle = 0
    
    # Delay time (50%)
    t_50_idx = np.where(y >= 0.5 * y_final)[0]
    t_delay = t[t_50_idx[0]] if len(t_50_idx) > 0 else 0
    
    # --- Panel 1: Step Response with Annotations ---
    ax = axes[0, 0]
    ax.plot(t * 1000, y, 'b-', linewidth=2, label='Step Response')
    ax.axhline(y=y_final, color='k', linestyle='--', alpha=0.5, label=f'Final Value: {y_final:.3f}')
    ax.axhline(y=1.02 * y_final, color='g', linestyle=':', alpha=0.5)
    ax.axhline(y=0.98 * y_final, color='g', linestyle=':', alpha=0.5, label='±2% Settling Band')
    
    # Mark key points
    ax.scatter([t_peak*1000], [y_peak], color='red', s=100, zorder=5, label=f'Peak: {y_peak:.3f}')
    ax.axvline(x=t_settle*1000, color='orange', linestyle='--', alpha=0.7, 
               label=f'Settling Time: {t_settle*1000:.0f}ms')
    
    # Shade rise time region
    ax.axvspan(t_10*1000, t_90*1000, alpha=0.2, color='yellow', label=f'Rise Time: {t_rise*1000:.0f}ms')
    
    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('Angle (normalized)', fontsize=11)
    ax.set_title('Closed-Loop Step Response', fontsize=12, fontweight='bold')
    ax.legend(loc='right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 1500])
    ax.set_ylim([0, max(y_peak * 1.1, 1.2)])
    
    # --- Panel 2: Metrics Summary ---
    ax = axes[0, 1]
    ax.axis('off')
    
    metrics_text = f"""
    ╔═══════════════════════════════════════════════╗
    ║       TRANSIENT RESPONSE METRICS              ║
    ╠═══════════════════════════════════════════════╣
    ║                                               ║
    ║  Rise Time (10-90%):    {t_rise*1000:>8.1f} ms          ║
    ║                                               ║
    ║  Peak Time:             {t_peak*1000:>8.1f} ms          ║
    ║                                               ║
    ║  Peak Overshoot:        {overshoot_pct:>8.1f} %           ║
    ║                                               ║
    ║  Settling Time (2%):    {t_settle*1000:>8.1f} ms          ║
    ║                                               ║
    ║  Delay Time (50%):      {t_delay*1000:>8.1f} ms          ║
    ║                                               ║
    ║  Final Value:           {y_final:>8.4f}              ║
    ║                                               ║
    ╚═══════════════════════════════════════════════╝
    """
    
    ax.text(0.1, 0.9, metrics_text, transform=ax.transAxes, fontsize=11,
            fontfamily='monospace', verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
    
    # --- Panel 3: Response Breakdown ---
    ax = axes[1, 0]
    
    # Show different phases of response
    phase1_end = t_90_idx[0] if len(t_90_idx) > 0 else len(t)//4
    phase2_end = np.argmax(y)
    phase3_end = settled_idx[-1] if len(settled_idx) > 0 else len(t)
    
    ax.fill_between(t[:phase1_end]*1000, y[:phase1_end], alpha=0.4, color='yellow', label='Rise Phase')
    ax.fill_between(t[phase1_end:phase2_end]*1000, y[phase1_end:phase2_end], alpha=0.4, color='red', label='Overshoot Phase')
    ax.fill_between(t[phase2_end:phase3_end]*1000, y[phase2_end:phase3_end], alpha=0.4, color='orange', label='Settling Phase')
    ax.fill_between(t[phase3_end:]*1000, y[phase3_end:], alpha=0.4, color='green', label='Steady State')
    
    ax.plot(t * 1000, y, 'b-', linewidth=2)
    ax.axhline(y=y_final, color='k', linestyle='--', alpha=0.5)
    
    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('Angle (normalized)', fontsize=11)
    ax.set_title('Response Phases', fontsize=12, fontweight='bold')
    ax.legend(loc='right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 1500])
    
    # --- Panel 4: Complete Response (Extended Time) ---
    ax = axes[1, 1]
    t_long, y_long = signal.step(T_angle, T=np.linspace(0, 5, 5000))
    ax.plot(t_long * 1000, y_long, 'b-', linewidth=2)
    ax.axhline(y=y_final, color='k', linestyle='--', alpha=0.5, label='Final Value')
    ax.axhline(y=1.02 * y_final, color='g', linestyle=':', alpha=0.5)
    ax.axhline(y=0.98 * y_final, color='g', linestyle=':', alpha=0.5)
    
    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('Angle (normalized)', fontsize=11)
    ax.set_title('Extended Step Response (5 seconds)', fontsize=12, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {save_path}")
    
    return {
        't_rise': t_rise,
        't_peak': t_peak,
        't_settle': t_settle,
        'overshoot': overshoot_pct,
        'y_final': y_final
    }

# =============================================================================
# FIGURE 5: SETTLING TIME ANALYSIS
# =============================================================================

def plot_settling_time(sys_dict, save_path):
    """Detailed settling time analysis with multiple criteria."""
    
    T_angle = sys_dict['angle_closed']
    t, y = signal.step(T_angle, T=np.linspace(0, 3, 3000))
    y_final = y[-1]
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Calculate settling times for different criteria
    criteria = [1, 2, 5, 10]  # Percent
    settling_times = {}
    
    for pct in criteria:
        band = pct / 100 * y_final
        settled_idx = np.where(np.abs(y - y_final) > band)[0]
        if len(settled_idx) > 0:
            settling_times[pct] = t[settled_idx[-1]]
        else:
            settling_times[pct] = 0
    
    # --- Panel 1: Settling with Different Bands ---
    ax = axes[0, 0]
    ax.plot(t * 1000, y, 'b-', linewidth=2, label='Step Response')
    ax.axhline(y=y_final, color='k', linestyle='--', alpha=0.5)
    
    colors = ['red', 'orange', 'yellow', 'green']
    for (pct, color) in zip(criteria, colors):
        band = pct / 100 * y_final
        ax.axhline(y=y_final + band, color=color, linestyle=':', alpha=0.7)
        ax.axhline(y=y_final - band, color=color, linestyle=':', alpha=0.7)
        ax.axvline(x=settling_times[pct]*1000, color=color, linestyle='--', alpha=0.7,
                   label=f'{pct}%: {settling_times[pct]*1000:.0f}ms')
    
    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('Angle (normalized)', fontsize=11)
    ax.set_title('Settling Time by Criteria', fontsize=12, fontweight='bold')
    ax.legend(loc='right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 2000])
    
    # --- Panel 2: Settling Times Bar Chart ---
    ax = axes[0, 1]
    bars = ax.bar([f'{pct}%' for pct in criteria], 
                  [settling_times[pct]*1000 for pct in criteria],
                  color=colors, edgecolor='black')
    ax.set_xlabel('Settling Criterion', fontsize=11)
    ax.set_ylabel('Settling Time (ms)', fontsize=11)
    ax.set_title('Comparison of Settling Criteria', fontsize=12, fontweight='bold')
    
    for bar, pct in zip(bars, criteria):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 10,
                f'{settling_times[pct]*1000:.0f}ms', ha='center', fontsize=11, fontweight='bold')
    
    ax.grid(True, alpha=0.3, axis='y')
    
    # --- Panel 3: Error vs Time ---
    ax = axes[1, 0]
    error = np.abs(y - y_final) / y_final * 100  # Percent error
    ax.plot(t * 1000, error, 'r-', linewidth=2)
    
    for (pct, color) in zip(criteria, colors):
        ax.axhline(y=pct, color=color, linestyle='--', alpha=0.7, label=f'{pct}% threshold')
    
    ax.set_xlabel('Time (ms)', fontsize=11)
    ax.set_ylabel('Error (%)', fontsize=11)
    ax.set_title('Error vs Time', fontsize=12, fontweight='bold')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_xlim([0, 2000])
    ax.set_ylim([0, max(error[10:]) * 1.1])  # Skip initial transient
    ax.set_yscale('log')
    
    # --- Panel 4: Summary ---
    ax = axes[1, 1]
    ax.axis('off')
    
    # Get poles for theoretical calculation
    poles = np.roots(T_angle.den)
    complex_poles = [p for p in poles if np.abs(np.imag(p)) > 1e-6]
    
    if len(complex_poles) > 0:
        dominant = max(complex_poles, key=lambda p: np.real(p))
        wn = np.abs(dominant)
        zeta = -np.real(dominant) / wn
        sigma = -np.real(dominant)
        
        # Theoretical settling time for 2%: ts ≈ 4/(ζωn) = 4/σ
        ts_theoretical_2pct = 4 / sigma
        ts_theoretical_5pct = 3 / sigma
    else:
        zeta = 1.0
        wn = 0
        ts_theoretical_2pct = 0
        ts_theoretical_5pct = 0
    
    summary_text = f"""
    ╔═══════════════════════════════════════════════════════╗
    ║            SETTLING TIME ANALYSIS                     ║
    ╠═══════════════════════════════════════════════════════╣
    ║                                                       ║
    ║  MEASURED SETTLING TIMES:                             ║
    ║  ─────────────────────────                            ║
    ║    1% criterion:   {settling_times[1]*1000:>8.1f} ms                      ║
    ║    2% criterion:   {settling_times[2]*1000:>8.1f} ms  ← Standard          ║
    ║    5% criterion:   {settling_times[5]*1000:>8.1f} ms                      ║
    ║   10% criterion:   {settling_times[10]*1000:>8.1f} ms                      ║
    ║                                                       ║
    ║  THEORETICAL (from dominant poles):                   ║
    ║  ─────────────────────────────────                    ║
    ║    ts ≈ 4/(ζωn) = 4/σ                                 ║
    ║    2% theoretical: {ts_theoretical_2pct*1000:>8.1f} ms                      ║
    ║    5% theoretical: {ts_theoretical_5pct*1000:>8.1f} ms                      ║
    ║                                                       ║
    ║  Damping ratio ζ = {zeta:.3f}                              ║
    ║  Natural freq ωn = {wn:.2f} rad/s                       ║
    ║                                                       ║
    ╚═══════════════════════════════════════════════════════╝
    """
    
    ax.text(0.05, 0.95, summary_text, transform=ax.transAxes, fontsize=10,
            fontfamily='monospace', verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.3))
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {save_path}")
    
    return settling_times

# =============================================================================
# MAIN EXECUTION
# =============================================================================

if __name__ == "__main__":
    import warnings
    warnings.filterwarnings('ignore', 'Badly conditioned')
    
    output_dir = '/home/maddy/Documents/Rate_Mode/ESP32-Quadcopter-Firmware-bd3f9386d99e08d1fffb3143f7a688fa85db2e28/analysis_output'
    
    print("="*60)
    print("QUADCOPTER CONTROL SYSTEM STABILITY ANALYSIS")
    print("="*60)
    print(f"\nPID Configuration:")
    print(f"  Rate Loop:  Kp={RATE_KP}, Ki={RATE_KI}, Kd={RATE_KD}")
    print(f"  Angle Loop: Kp={ANGLE_KP}, Ki={ANGLE_KI}, Kd={ANGLE_KD}")
    print(f"  Motor τ = {TAU_MOTOR*1000}ms, Torque Gain = {TORQUE_GAIN}")
    print()
    
    print("Building control system model...")
    sys_dict = build_system()
    
    print("\nGenerating analysis plots...")
    
    # Generate all plots
    plot_splane(sys_dict, f'{output_dir}/1_splane_analysis.png')
    
    stability_type = plot_stability_classification(sys_dict, f'{output_dir}/2_stability_classification.png')
    
    dom_zeta, dom_wn = plot_damping_analysis(sys_dict, f'{output_dir}/3_damping_analysis.png')
    
    transient = plot_transient_response(sys_dict, f'{output_dir}/4_transient_response.png')
    
    settling = plot_settling_time(sys_dict, f'{output_dir}/5_settling_time.png')
    
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"\n  Stability:        {stability_type}")
    print(f"  Damping Ratio:    ζ = {dom_zeta:.3f}")
    print(f"  Natural Freq:     ωn = {dom_wn:.2f} rad/s ({dom_wn/(2*np.pi):.2f} Hz)")
    print(f"  Rise Time:        {transient['t_rise']*1000:.1f} ms")
    print(f"  Settling Time:    {settling[2]*1000:.1f} ms (2% criterion)")
    print(f"  Overshoot:        {transient['overshoot']:.1f}%")
    
    print(f"\n  All plots saved to: {output_dir}/")
    print("="*60)
