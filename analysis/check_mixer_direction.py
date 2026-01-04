import pandas as pd
import numpy as np
import sys

def analyze_mixer(file_path):
    try:
        df = pd.read_csv(file_path)
    except Exception as e:
        print(f"Error reading file: {e}")
        return

    # Filter for active mixer
    df = df[df['rc_thr'] > 1150]
    
    if len(df) == 0:
        print("No active flight data (throttle > 1150) found.")
        return

    # Motors: 
    # M1: Rear Right (CCW)
    # M2: Front Right (CW)
    # M3: Rear Left (CW)
    # M4: Front Left (CCW)
    
    # Pitch Action: Rear (M1+M3) vs Front (M2+M4)
    # Positive Pitch PID -> Typically means "Lift Nose" (if acting on error) or "Push Nose Down" (if acting on measurement).
    # Let's check what the motors actually DID relative to the PID value.
    
    rear_motors = df['m1'] + df['m3']
    front_motors = df['m2'] + df['m4']
    
    # Diff > 0 means Rear is working harder than Front
    pitch_motor_diff = rear_motors - front_motors
    
    # Correlation
    corr = df['pid_pitch'].corr(pitch_motor_diff)
    
    print(f"File: {file_path}")
    print(f"Data Points: {len(df)}")
    print(f"Correlation (PID_Pitch vs Rear-Front Diff): {corr:.4f}")
    
    # Interpretation
    print("\nInterpretation:")
    print("If Correlation is NEGATIVE: As PID increases (goes positive), Rear motors DECREASE relative to Front.")
    print("If Correlation is POSITIVE: As PID increases (goes positive), Rear motors INCREASE relative to Front.")
    
    # Check a specific high-pitch moment if possible
    # Find max pitch angle
    max_pitch_idx = df['pitch'].idxmax()
    row = df.loc[max_pitch_idx]
    
    print("\nState at Max Pitch:")
    print(f" Pitch: {row['pitch']:.2f} deg")
    print(f" PID_Pitch: {row['pid_pitch']:.2f}")
    print(f" M1(RR): {row['m1']}  M2(FR): {row['m2']}")
    print(f" M3(RL): {row['m3']}  M4(FL): {row['m4']}")
    print(f" Rear Avg: {(row['m1']+row['m3'])/2:.1f}  Front Avg: {(row['m2']+row['m4'])/2:.1f}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 check_mixer.py <csv_file>")
    else:
        analyze_mixer(sys.argv[1])
