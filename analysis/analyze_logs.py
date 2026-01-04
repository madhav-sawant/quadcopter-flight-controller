import os
import pandas as pd
import numpy as np

LOG_DIR = "/home/maddy/Videos/Firmware/Blackbox "

def analyze_session(session_path):
    print(f"Analyzing session: {session_path}")
    files = sorted([f for f in os.listdir(session_path) if f.endswith(".csv")])
    if not files:
        print("  No CSV files found.")
        return

    # Aggregate summaries
    pitch_errs = []
    motor_sats = []
    flags_triggered = set()
    
    for f in files:
        fpath = os.path.join(session_path, f)
        try:
            df = pd.read_csv(fpath)
        except Exception as e:
            print(f"  Error reading {f}: {e}")
            continue
            
        if df.empty:
            continue

        # Convert columns to numeric, forcing errors to NaN
        cols = ['pitch', 'angle_sp_pitch', 'm1', 'm2', 'm3', 'm4', 'flags', 'accel_x', 'accel_y', 'accel_z']
        for c in cols:
            if c in df.columns:
                df[c] = pd.to_numeric(df[c], errors='coerce')
        
        # 1. Pitch Analysis
        # Error = Setpoint - Actual
        # If drifting backward (Nose Up -> Positive Pitch is typically backward in standard configs? Wait. 
        # Check sign convention from logs: if drifting backward, and user says "move backward without stick input", 
        # usually means the physical drone is pitched up.
        # But let's check the error term.
        # If Angle Mode is working, it should try to zero the error.
        
        if 'pitch' in df.columns and 'angle_sp_pitch' in df.columns:
            df['pitch_err'] = df['angle_sp_pitch'] - df['pitch']
            mean_err = df['pitch_err'].mean()
            # print(f"  {f}: Mean Pitch Err: {mean_err:.2f}")
            pitch_errs.append(mean_err)
        
        # 2. Motor Saturation
        # Check if motors hit 1000 (min) or max (depends on ESC, usually 2000 or PWM limit).
        # User says "Motors shut down mid-test". Look for sudden drop to 0 or 1000.
        if 'm1' in df.columns:
            # Assuming 1000 is min airmode/idle, or 0 is disarmed.
            # Check for max saturation
            max_out =  max(df['m1'].max(), df['m2'].max(), df['m3'].max(), df['m4'].max())
            min_out = min(df['m1'].min(), df['m2'].min(), df['m3'].min(), df['m4'].min())
            
            # Count saturation events (near 2000 or whatever max is, assume 1800+ is high)
            # Or distinct imbalance.
            # Motor Layout: Rear=M1/M3, Front=M2/M4.
            # Backward drift correction requires Front Low, Rear High (to pitch Nose Down).
            # If drifting Backward (Nose Up), Controller should be increasing Rear (M1/M3) and decreasing Front (M2/M4).
            
            avg_m1 = df['m1'].mean()
            avg_m2 = df['m2'].mean()
            avg_m3 = df['m3'].mean()
            avg_m4 = df['m4'].mean()
            
            rear_avg = (avg_m1 + avg_m3) / 2
            front_avg = (avg_m2 + avg_m4) / 2
            
            # print(f"  {f}: Front Avg: {front_avg:.0f}, Rear Avg: {rear_avg:.0f}")
            
        # 3. Flags
        if 'flags' in df.columns:
            unique_flags = df['flags'].unique()
            for fl in unique_flags:
                flags_triggered.add(fl)
    
    print("  Summary:")
    if pitch_errs:
        print(f"  Avg Pitch Error across files: {np.mean(pitch_errs):.4f}")
    print(f"  Flags Found: {flags_triggered}")
    # Correlation Check
    # Do we see a persistent positive pitch bias?
    # If Avg Pitch Error is positive, it means Setpoint > Pitch. (Use wants pitch X, drone is at X - err).
    # If Setpoint is 0, and Pitch is -5 (Nose Down), Error is +5. Controller wants to Pitch Up.
    # If drone drifts backward, it implies Pitch is > 0 (Nose Up). Setpoint 0. Error -5. Controller wants Nose Down.
    
    # We really need to know what "Backward" implies in this logs' sign convention.
    # Usually: Pitch + = Nose Up.
    # If drone moves backward, it is physically Nose Up.
    # If Controller sees Nose Up (Pitch > 0), it should command Rear > Front? No, to pitch DOWN, Rear > Front.
    # Wait. Torque = r x F. 
    # To Pitch DOWN (Nose Down), Rear Props must lift MORE (Up force on Rear).
    # So Rear > Front = Pitch Down.
    
    # If Log shows Pitch > 0, and Controller outputs Rear > Front, then Controller is TRYING to fix it.
    # If Log shows Pitch about 0, but drone goes backward, then Estimator is WRONG (Gravity vector misaligned).
    
    pass

def main():
    if not os.path.exists(LOG_DIR):
        print(f"Log directory not found: {LOG_DIR}")
        return

    subdirs = ['1', '2', '3', '4']
    for d in subdirs:
        path = os.path.join(LOG_DIR, d)
        if os.path.isdir(path):
            analyze_session(path)
        else:
            print(f"Session {d} not found.")

if __name__ == "__main__":
    main()
