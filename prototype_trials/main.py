import os
import sys
import time
import threading
import serial
import pygame
import statistics
import datetime
import pandas as pd

# Suppress video display requirement for Linux headless execution
os.environ["SDL_VIDEODRIVER"] = "dummy"

class RoboticLegController:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = False
        self.reader_thread = None
        
        # Deadzone threshold for Xbox analog sticks
        self.deadzone = 0.5
        
        # Track the active command for each joint so we can stop it on release
        self.j1_state = None
        self.j2_state = None
        self.j3_state = None
        
        # Vacuum and Experiment State tracking
        self.vacuum_on = False
        self.is_recording = False
        self.mute_serial_print = False  # Used to keep terminal clean while typing names
        
        self.baseline_voltages = []     # Rolling window of voltages before pump turns on
        self.exp_times = []             # Timestamps during current experiment
        self.exp_voltages = []          # Voltage readings during current experiment
        self.exp_start_time = 0

    def connect_serial(self):
        """Attempts to connect to the Arduino via Serial."""
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2) 
            print(f"[SUCCESS] Connected to Arduino on {self.port} at {self.baudrate} baud.")
            return True
        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {self.port}: {e}")
            return False

    def init_controller(self):
        """Initializes Pygame gamepad subsystem and connects Xbox controller."""
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("[ERROR] No Xbox controller found! Please check cable connection.")
            return False

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"[SUCCESS] Connected Xbox Controller: {self.joystick.get_name()}")
        return True

    def _read_serial_loop(self):
        """Background thread reading serial input (ADC values, status text)."""
        while self.running:
            if self.ser and self.ser.in_waiting > 0:
                try:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        # Print raw output unless we are typing an experiment name
                        if not self.mute_serial_print:
                            print(f"[ARDUINO] {line}")
                        
                        # Parse Voltage data if present in the string
                        if "Voltage =" in line:
                            parts = line.split("Voltage =")
                            v_str = parts[1].strip()
                            voltage = float(v_str)
                            
                            if not self.is_recording:
                                # Maintain a rolling baseline of the last 10 readings (~2 secs at 5Hz)
                                self.baseline_voltages.append(voltage)
                                if len(self.baseline_voltages) > 10:
                                    self.baseline_voltages.pop(0)
                            else:
                                # Record data during active experiment
                                self.exp_times.append(time.time() - self.exp_start_time)
                                self.exp_voltages.append(voltage)
                                
                except Exception as e:
                    print(f"[ERROR] Serial read error: {e}")
                    break
            time.sleep(0.01)

    def send_command(self, char_cmd):
        """Sends a single character command to Arduino."""
        if self.ser and self.ser.is_open:
            self.ser.write(char_cmd.encode('utf-8'))

    def update_joint_axis(self, axis_val, cmd_neg, cmd_pos, current_state):
        if axis_val < -self.deadzone:
            target_state = cmd_neg
        elif axis_val > self.deadzone:
            target_state = cmd_pos
        else:
            target_state = None
            
        if current_state != target_state:
            if target_state is not None:
                self.send_command(target_state)
            else:
                self.send_command(current_state)
            return target_state
        return current_state

    def process_and_save_experiment(self):
        """Processes the recorded data and prompts user to save it."""
        self.mute_serial_print = True  # Pause serial prints so the prompt isn't interrupted
        
        if len(self.exp_voltages) == 0:
            print("\n[LOG] No sensor data collected during that run.")
            self.mute_serial_print = False
            return
            
        # 1. Calculate Statistics
        baseline = sum(self.baseline_voltages) / len(self.baseline_voltages) if self.baseline_voltages else 5.0
        avg_v = sum(self.exp_voltages) / len(self.exp_voltages)
        drop_v = baseline - avg_v
        drop_pct = (drop_v / baseline) * 100 if baseline > 0 else 0
        std_v = statistics.stdev(self.exp_voltages) if len(self.exp_voltages) > 1 else 0.0
        
        # 2. Success Condition (Average < 3.9V)
        success = 'Y' if avg_v < 3.9 else 'N'
        
        # 3. Stabilization Time (Time taken to first drop below 3.9V)
        stab_time = None
        for t, v in zip(self.exp_times, self.exp_voltages):
            if v < 3.9:
                stab_time = t
                break
        stab_time_str = round(stab_time, 3) if stab_time is not None else "N/A"

        # Display Summary
        print(f"\n--- EXPERIMENT RESULTS ---")
        print(f"Baseline (Pump Off) : {baseline:.3f} V")
        print(f"Average Hold Volt   : {avg_v:.3f} V")
        print(f"Voltage Drop        : {drop_v:.3f} V ({drop_pct:.1f}%)")
        print(f"Standard Dev        : {std_v:.3f} V")
        print(f"Stabilize Time      : {stab_time_str} s")
        print(f"Adhesion Success    : {success}")
        print(f"--------------------------")

        # Prompt for save details
        print("\n--- EXPERIMENT LOGGING ---")
        name = input("Enter experiment name (or press Enter to skip saving): ").strip()
        
        if name:
            surf_type = input("Surface type (e.g., Glass, Wood, Plastic): ").strip()
            weight = input("Weight of lifted object (e.g., 50g, 0.5kg): ").strip()
            angle = input("Approach angle (e.g., 0, 45, 90): ").strip()
            curvature = input("Surface curvature (e.g., Flat, Convex, Concave): ").strip()
            wet = input("Wet surface (y/n): ").strip().upper()
            motor_fail = input("Motor failure (y/n): ").strip().upper()
            
            data = {
                "Date": [datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")],
                "Experiment Name": [name],
                "Surface Type": [surf_type],
                "Payload Weight": [weight],
                "Approach Angle": [angle],
                "Surface Curvature": [curvature],
                "Wet Surface": [wet],
                "Motor Failure": [motor_fail],
                "Baseline (V)": [round(baseline, 3)],
                "Average (V)": [round(avg_v, 3)],
                "Drop (V)": [round(drop_v, 3)],
                "Drop (%)": [round(drop_pct, 1)],
                "Std Dev (V)": [round(std_v, 3)],
                "Stabilize Time (s)": [stab_time_str],
                "Adhesion Success": [success]
            }
            
            file_name = "payload_experiments.xlsx"
            try:
                if os.path.exists(file_name):
                    df_existing = pd.read_excel(file_name)
                    df_new = pd.DataFrame(data)
                    df_final = pd.concat([df_existing, df_new], ignore_index=True)
                    df_final.to_excel(file_name, index=False)
                else:
                    df = pd.DataFrame(data)
                    df.to_excel(file_name, index=False)
                print(f"[SUCCESS] Saved data to '{file_name}' under '{name}'\n")
            except Exception as e:
                print(f"[ERROR] Could not save Excel file. Ensure it is not open in another program. Error: {e}\n")
        else:
            print("[LOG] Save skipped.\n")
            
        self.mute_serial_print = False # Resume standard serial prints

    def run(self):
        """Main execution loop for gamepad event mapping."""
        if not self.connect_serial() or not self.init_controller():
            return

        self.running = True
        self.reader_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
        self.reader_thread.start()

        print("\n" + "="*50)
        print("       ROBOTIC LEG XBOX CONTROLLER ACTIVE")
        print("="*50)
        print(" CONTROLS:")
        print(" - Joint 1: Left Stick Left/Right, X Button (Home)")
        print(" - Joint 2: Left Stick Up/Down (Inverted), Y Button (Home)")
        print(" - Joint 3: Right Stick Up/Down, B Button (Home)")
        print(" - Vacuum : A Button (Toggle ON/OFF & Trigger Data Log)")
        print(" - Exit   : Press Ctrl+C")
        print("="*50 + "\n")

        try:
            while self.running:
                pygame.event.pump()

                for event in pygame.event.get():
                    if event.type == pygame.JOYBUTTONDOWN:
                        if event.button == 0:    # A Button - Vacuum Toggle & Experiment
                            self.vacuum_on = not self.vacuum_on
                            self.send_command('V' if self.vacuum_on else 'B')
                            
                            if self.vacuum_on:
                                # Start recording
                                self.exp_times.clear()
                                self.exp_voltages.clear()
                                self.exp_start_time = time.time()
                                self.is_recording = True
                                print("\n[LOG] Vacuum ON: Experiment started. Recording data...")
                            else:
                                # Stop recording and process in background thread
                                self.is_recording = False
                                threading.Thread(target=self.process_and_save_experiment, daemon=True).start()
                                
                        elif event.button == 1:  # B Button 
                            self.send_command('x')
                        elif event.button == 2:  # X Button 
                            self.send_command('w')
                        elif event.button == 3:  # Y Button 
                            self.send_command('s')

                # --- Process Analog Stick Axes ---
                ls_x = self.joystick.get_axis(0)
                self.j1_state = self.update_joint_axis(ls_x, 'q', 'e', self.j1_state)

                ls_y = self.joystick.get_axis(1)
                self.j2_state = self.update_joint_axis(ls_y, 'd', 'a', self.j2_state)

                rs_y_idx = 3 if self.joystick.get_numaxes() <= 4 else 4
                rs_y = self.joystick.get_axis(rs_y_idx)
                self.j3_state = self.update_joint_axis(rs_y, 'z', 'c', self.j3_state)

                time.sleep(0.02) 

        except KeyboardInterrupt:
            print("\nShutting down Robotic Leg interface...")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        pygame.quit()
        print("System safely shut down.")

if __name__ == "__main__":
    controller = RoboticLegController(port="/dev/ttyACM0", baudrate=115200)
    controller.run()
