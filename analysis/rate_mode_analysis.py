#!/usr/bin/env python3
"""
Rate Mode Stability Analysis - RESEARCH PAPER VERSION (B&W)
Author: Madhav Sawant
Only generates Step Response and Bode Plot (essential graphs)
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import os

# Output directory
output_dir = '/home/maddy/Videos/Firmware/docs/graphs'
os.makedirs(output_dir, exist_ok=True)

# Set publication style - BLACK AND WHITE
plt.rcParams.update({
    'figure.figsize': (8, 6),
    'font.size': 11,
    'font.family': 'serif',
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'lines.linewidth': 1.5,
    'axes.linewidth': 0.8,
    'grid.linewidth': 0.5,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
})

#=============================================================================
# SYSTEM PARAMETERS
#=============================================================================
print("="*60)
print("RATE MODE STABILITY ANALYSIS - B&W PAPER VERSION")
print("="*60)

K_tau = 85.0
tau_m = 0.030
Kp_rate = 0.40
Ki_rate = 0.20
Kd_rate = 0.03
alpha_d = 0.42
dt = 0.004
Td = dt * (1 - alpha_d) / alpha_d

print(f"\nRate PID: Kp={Kp_rate}, Ki={Ki_rate}, Kd={Kd_rate}")

#=============================================================================
# TRANSFER FUNCTIONS
#=============================================================================
plant_num = [K_tau]
plant_den = [tau_m, 1, 0]
pid_num = [(Kp_rate*Td + Kd_rate), (Kp_rate + Ki_rate*Td), Ki_rate]
pid_den = [Td, 1, 0]

open_loop_num = np.polymul(plant_num, pid_num)
open_loop_den = np.polymul(plant_den, pid_den)
closed_loop_num = open_loop_num
closed_loop_den = np.polyadd(open_loop_den, open_loop_num)

rate_closed_loop = signal.TransferFunction(closed_loop_num, closed_loop_den)
rate_open_loop = signal.TransferFunction(open_loop_num, open_loop_den)
rate_poles = np.roots(closed_loop_den)

# Get damping ratio
for pole in rate_poles:
    if np.abs(pole.imag) > 1e-6 and pole.imag > 0:
        wn = np.abs(pole)
        zeta = -np.real(pole) / wn
        print(f"Dominant poles: ζ={zeta:.2f}, ωn={wn:.0f} rad/s")
        break

print("\nGenerating essential graphs only...")

#=============================================================================
# GRAPH 1: STEP RESPONSE (B&W)
#=============================================================================
fig, ax = plt.subplots(figsize=(8, 5))

t = np.linspace(0, 0.4, 1000)
t_out, y_out = signal.step(rate_closed_loop, T=t)

ax.plot(t_out * 1000, y_out, 'k-', linewidth=1.5, label='Step Response')
ax.axhline(y=1, color='black', linestyle='--', linewidth=1, label='Reference (1.0)')
ax.axhline(y=0.98, color='gray', linestyle=':', linewidth=0.8, label='±2% Band')
ax.axhline(y=1.02, color='gray', linestyle=':', linewidth=0.8)

# Metrics
final_value = y_out[-1]
rise_idx = np.where(y_out >= 0.9 * final_value)[0]
rise_time = t_out[rise_idx[0]] if len(rise_idx) > 0 else 0
settle_idx = np.where(np.abs(y_out - final_value) <= 0.02 * final_value)[0]
settling_time = t_out[settle_idx[0]] if len(settle_idx) > 0 else 0
overshoot = (np.max(y_out) - final_value) / final_value * 100

ax.plot(rise_time * 1000, 0.9, 'ko', markersize=6)
ax.annotate(f'Rise Time\n{rise_time*1000:.0f} ms', xy=(rise_time*1000, 0.5), 
           xytext=(rise_time*1000 + 40, 0.45), fontsize=10,
           bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray'))

ax.plot(settling_time * 1000, 0.98, 'ks', markersize=6, markerfacecolor='white')
ax.annotate(f'Settling\n{settling_time*1000:.0f} ms', xy=(settling_time*1000, 0.98), 
           xytext=(settling_time*1000 + 30, 0.82), fontsize=10,
           bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray'))

ax.set_xlabel('Time (ms)', fontsize=12)
ax.set_ylabel('Rate Response (normalized)', fontsize=12)
ax.set_title('Rate Mode: Unit Step Response', fontsize=14, fontweight='bold')
ax.legend(loc='lower right', framealpha=0.95, fontsize=10)
ax.set_xlim([0, 400])
ax.set_ylim([0, 1.15])

plt.tight_layout()
plt.savefig(f'{output_dir}/rate_mode_step_response.png', dpi=300, facecolor='white')
print(f"  ✓ rate_mode_step_response.png")
plt.close()

print(f"    Rise Time: {rise_time*1000:.0f} ms, Settling: {settling_time*1000:.0f} ms")

#=============================================================================
# GRAPH 2: BODE PLOT (B&W)
#=============================================================================
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)

w, mag, phase = signal.bode(rate_open_loop, np.logspace(-1, 4, 1000))

# Magnitude
ax1.semilogx(w, mag, 'k-', linewidth=1.5, label='Open-Loop Magnitude')
ax1.axhline(y=0, color='black', linestyle='--', linewidth=1, label='0 dB')
ax1.set_ylabel('Magnitude (dB)', fontsize=12)
ax1.set_title('Rate Mode: Open-Loop Bode Plot', fontsize=14, fontweight='bold')
ax1.legend(loc='upper right', framealpha=0.95, fontsize=10)
ax1.set_xlim([0.1, 10000])

crossover_idx = np.where(np.diff(np.sign(mag)))[0]
if len(crossover_idx) > 0:
    wgc = w[crossover_idx[0]]
    phase_at_wgc = phase[crossover_idx[0]]
    phase_margin = 180 + phase_at_wgc
    
    ax1.axvline(x=wgc, color='gray', linestyle=':', linewidth=1)
    ax1.plot(wgc, 0, 'ko', markersize=7)
    ax1.annotate(f'ωgc = {wgc:.0f} rad/s', xy=(wgc, 5), xytext=(wgc*2, 20), fontsize=10,
                arrowprops=dict(arrowstyle='->', color='black', lw=1))

# Phase
ax2.semilogx(w, phase, 'k-', linewidth=1.5, label='Open-Loop Phase')
ax2.axhline(y=-180, color='black', linestyle='--', linewidth=1, label='-180°')
ax2.set_xlabel('Frequency (rad/s)', fontsize=12)
ax2.set_ylabel('Phase (degrees)', fontsize=12)
ax2.legend(loc='lower left', framealpha=0.95, fontsize=10)  # Moved to lower left
ax2.set_xlim([0.1, 10000])
ax2.set_ylim([-200, -60])  # Proper scale for PM visualization

if len(crossover_idx) > 0:
    ax2.axvline(x=wgc, color='gray', linestyle=':', linewidth=1)
    ax2.plot(wgc, phase_at_wgc, 'ko', markersize=7)
    
    # Simple, clean PM dimension line (offset to right of crossover)
    visual_x = wgc * 2.5
    
    # Horizontal guide lines from crossover point to dimension line
    ax2.plot([wgc, visual_x], [phase_at_wgc, phase_at_wgc], 'k:', linewidth=0.8)
    ax2.plot([wgc, visual_x], [-180, -180], 'k:', linewidth=0.8)
    
    # Double-headed arrow from -180° to phase_at_wgc (showing PM span)
    ax2.annotate('', xy=(visual_x, -180), xytext=(visual_x, phase_at_wgc),
                arrowprops=dict(arrowstyle='<->', color='black', lw=1.5))
    
    # PM label to the right of the arrow
    mid_phase = (phase_at_wgc + -180) / 2
    ax2.text(visual_x * 1.3, mid_phase, f'PM = {phase_margin:.0f}°', 
             fontsize=11, fontweight='bold', va='center', ha='left',
             bbox=dict(boxstyle='round', facecolor='white', edgecolor='gray'))

plt.tight_layout()
plt.savefig(f'{output_dir}/rate_mode_bode.png', dpi=300, facecolor='white')
print(f"  ✓ rate_mode_bode.png")
plt.close()

print(f"    Phase Margin: {phase_margin:.0f}°")

#=============================================================================
# GRAPH 3: S-PLANE POLE-ZERO MAP (Rate Mode)
#=============================================================================
print("\nGenerating specialized graphs...")
fig, ax = plt.subplots(figsize=(8, 6))

# Plot all poles
for pole in rate_poles:
    if np.abs(pole.imag) < 1e-6:
        ax.plot(pole.real, 0, 'ks', markersize=8, markerfacecolor='white', markeredgewidth=2)
    else:
        ax.plot(pole.real, pole.imag, 'kx', markersize=10, markeredgewidth=2)

# Damping ratio lines for rate mode (0.707)
damping_data = [
    (0.707, '-.', 45),
    (0.5, ':', 30)
]
for zeta_line, ls, angle in damping_data:
    theta = np.arccos(zeta_line)
    r = np.linspace(0, 150, 50)
    ax.plot(-r * np.cos(theta), r * np.sin(theta), color='gray', linestyle=ls, linewidth=1)
    ax.plot(-r * np.cos(theta), -r * np.sin(theta), color='gray', linestyle=ls, linewidth=1)
    ax.text(-120, 120 * np.tan(theta)*0.9, f'$\\zeta={zeta_line}$', fontsize=9, color='gray')

# Annotate dominant poles
for pole in rate_poles:
    if np.abs(pole.imag) > 1e-6 and pole.imag > 0:
        wn_p = np.abs(pole)
        zeta_p = -np.real(pole) / wn_p
        ax.annotate(f'Dominant Poles\n$(\\zeta={zeta_p:.2f}, \\omega_n={wn_p:.0f})$', 
                   xy=(pole.real, pole.imag), 
                   xytext=(-50, 60),
                   fontsize=10, ha='center',
                   arrowprops=dict(arrowstyle='->', color='black', lw=1))
        break

ax.axvline(x=0, color='black', linestyle='-', linewidth=1)
ax.axhline(y=0, color='black', linestyle='-', linewidth=1)
ax.set_xlabel('Real Axis')
ax.set_ylabel('Imaginary Axis')
ax.set_title('Rate Mode: S-Plane Pole Locations')
ax.set_xlim([-140, 20])
ax.set_ylim([-130, 130])
ax.grid(True, linestyle=':', alpha=0.6)

plt.tight_layout()
plt.savefig(f'{output_dir}/rate_mode_damping.png', dpi=300)
print(f"  ✓ rate_mode_damping.png")
plt.close()



print("\n" + "="*60)
print(f"Rate Mode: ζ={zeta:.2f}, PM={phase_margin:.0f}°, Rise={rise_time*1000:.0f}ms")
print("="*60)
print("Done!")
