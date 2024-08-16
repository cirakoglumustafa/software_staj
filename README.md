# software_staj
aurdino codes: 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  80
#define SERVOMAX  600

#define SER0  0
#define SER1  1
#define SER2  2
#define SER3  3
#define SER4  4
#define SER5  5

int pwm0 = 380;
int pwm1 = 180;
int pwm2 = 300;
int pwm3 = 190;
int pwm4 = 300;
int pwm5 = 400;

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 Servo Test");
  pca9685.begin();
  pca9685.setPWMFreq(50);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any extra whitespace or newline characters

    int separatorIndex = input.indexOf(' ');
    if (separatorIndex != -1) {
      int servoIndex = input.substring(0, separatorIndex).toInt();
      int pwmValue = input.substring(separatorIndex + 1).toInt();

      if (pwmValue >= SERVOMIN && pwmValue <= SERVOMAX) {
        switch (servoIndex) {
          case 0:
            pwm0 = pwmValue;
            pca9685.setPWM(SER0, 0, pwm0);
            Serial.print("SER0 set to: "); Serial.println(pwm0);
            break;
          case 1:
            pwm1 = pwmValue;
            pca9685.setPWM(SER1, 0, pwm1);
            Serial.print("SER1 set to: "); Serial.println(pwm1);
            break;
          case 2:
            pwm2 = pwmValue;
            pca9685.setPWM(SER2, 0, pwm2);
            Serial.print("SER2 set to: "); Serial.println(pwm2);
            break;
          case 3:
            pwm3 = pwmValue;
            pca9685.setPWM(SER3, 0, pwm3);
            Serial.print("SER3 set to: "); Serial.println(pwm3);
            break;
          case 4:
            pwm4 = pwmValue;
            pca9685.setPWM(SER4, 0, pwm4);
            Serial.print("SER4 set to: "); Serial.println(pwm4);
            break;
          case 5:
            pwm5 = pwmValue;
            pca9685.setPWM(SER5, 0, pwm5);
            Serial.print("SER5 set to: "); Serial.println(pwm5);
            break;
          default:
            Serial.println("Invalid servo index. Use 0-5.");
        }
      } else {
        Serial.print("Invalid PWM value. Use a value between ");
        Serial.print(SERVOMIN);
        Serial.print(" and ");
        Serial.println(SERVOMAX);
      }
    } else {
      Serial.println("Invalid input format. Use: <servo index> <PWM value>");
    }
  }
}

python codes: 
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import serial
import time
import json
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from threading import Thread
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

class RobotArmController:
    SERVOMIN = 80
    SERVOMAX = 600
    STEPS = 100  # Number of steps for smooth interpolation
    DELAY = 0.01  # Delay between steps for smooth movement

    def __init__(self, port, baudrate):
        self.ser = self.initialize_serial(port, baudrate)
        self.saved_positions = []
        self.movement_history = []
        self.macros = {}
        self.notes = {}
        self.root = tk.Tk()
        self.root.title("Advanced Robot Arm Control Panel")
        self.root.geometry("1600x900")
        self.root.configure(background='#282C34')
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TFrame', background='#282C34')
        self.style.configure('TLabel', background='#ABB2BF', font=('Arial', 10))
        self.style.configure('TButton', background='#61AFEF', foreground='#282C34', font=('Arial', 10, 'bold'))
        self.sliders = []
        self.movement_mode = tk.StringVar(value="Linear")
        self.create_sensors()  # Sensörlerin oluşturulması GUI kurulumundan önce çağrılıyor
        self.setup_gui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def initialize_serial(self, port, baudrate):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            return ser
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Could not open serial port: {e}")
            return None

    def set_servo(self, servo_index, pwm_value):
        if self.ser:
            command = f"{servo_index} {pwm_value}\n"
            self.ser.write(command.encode())
            print(f"Sent: {command}")

    def on_slider_change(self, event, slider, servo_index):
        pwm_value = int(slider.get())
        self.set_servo(servo_index, pwm_value)
        self.update_graph()
        self.update_servo_status()
        self.update_simulation()

    def reset_servos(self):
        for i in range(6):
            mid_value = (self.SERVOMAX - self.SERVOMIN) // 2
            self.sliders[i].set(mid_value)
            self.set_servo(i, mid_value)
        self.update_graph()
        self.update_servo_status()
        self.update_simulation()

    def preset_position(self, positions):
        for i in range(6):
            self.sliders[i].set(positions[i])
            self.set_servo(i, positions[i])
        self.update_graph()
        self.update_servo_status()
        self.update_simulation()

    def save_position(self):
        position = [int(slider.get()) for slider in self.sliders]
        self.saved_positions.append(position)
        self.update_saved_positions_list()

    def update_saved_positions_list(self):
        self.saved_positions_list.delete(0, tk.END)
        for idx, pos in enumerate(self.saved_positions):
            note = self.notes.get(idx, "")
            self.saved_positions_list.insert(tk.END, f"Position {idx+1}: {pos} {note}")

    def go_to_position(self, index):
        if 0 <= index < len(self.saved_positions):
            position = self.saved_positions[index]
            self.move_to_position(position)
            self.movement_history.append(position)
            self.update_movement_history_list()

    def interpolate_positions(self, start_pos, end_pos, steps):
        delta = [(end - start) / steps for start, end in zip(start_pos, end_pos)]
        positions = []
        for step in range(steps):
            positions.append([int(start + step * d) for start, d in zip(start_pos, delta)])
        return positions

    def move_to_position(self, target_position):
        current_position = [int(slider.get()) for slider in self.sliders]
        positions = self.interpolate_positions(current_position, target_position, self.STEPS)
        
        if self.movement_mode.get() == "Linear":
            for position in positions:
                for i in range(6):
                    self.sliders[i].set(position[i])
                    self.set_servo(i, position[i])
                time.sleep(self.DELAY)
        elif self.movement_mode.get() == "Circular":
            center_position = [(self.SERVOMAX + self.SERVOMIN) // 2] * 6
            for position in positions:
                for i in range(6):
                    angle = np.radians(position[i] - center_position[i])
                    new_position = center_position[i] + int((target_position[i] - center_position[i]) * np.sin(angle))
                    self.sliders[i].set(new_position)
                    self.set_servo(i, new_position)
                time.sleep(self.DELAY)

        self.update_graph()
        self.update_servo_status()
        self.update_simulation()

    def execute_positions(self):
        for position in self.saved_positions:
            self.move_to_position(position)
            time.sleep(1)  # Pause between positions
        messagebox.showinfo("Info", "All positions executed.")

    def save_positions_to_file(self):
        filepath = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if filepath:
            with open(filepath, 'w') as file:
                json.dump(self.saved_positions, file)
            messagebox.showinfo("Info", f"Positions saved to {filepath}")

    def load_positions_from_file(self):
        filepath = filedialog.askopenfilename(defaultextension=".json", filetypes=[("JSON files", "*.json"), ("All files", "*.*")])
        if filepath:
            try:
                with open(filepath, 'r') as file:
                    self.saved_positions = json.load(file)
                self.update_saved_positions_list()
                messagebox.showinfo("Info", f"Positions loaded from {filepath}")
            except FileNotFoundError:
                messagebox.showwarning("Warning", "No positions file found.")
    
    def save_macro(self):
        macro_name = self.macro_name_entry.get()
        if macro_name:
            self.macros[macro_name] = self.saved_positions.copy()
            self.update_macro_list()
            self.saved_positions.clear()
            self.update_saved_positions_list()
            messagebox.showinfo("Info", f"Macro '{macro_name}' saved.")

    def load_macro(self, macro_name):
        if macro_name in self.macros:
            self.saved_positions = self.macros[macro_name].copy()
            self.update_saved_positions_list()
            messagebox.showinfo("Info", f"Macro '{macro_name}' loaded.")
    
    def update_macro_list(self):
        self.macro_listbox.delete(0, tk.END)
        for macro in self.macros:
            self.macro_listbox.insert(tk.END, macro)
    
    def save_note_for_position(self):
        selected_idx = self.saved_positions_list.curselection()
        if selected_idx:
            idx = selected_idx[0]
            note = self.note_entry.get()
            self.notes[idx] = note
            self.update_saved_positions_list()
            messagebox.showinfo("Info", f"Note saved for Position {idx+1}.")

    def auto_calibrate_servos(self):
        for i in range(6):
            self.set_servo(i, self.SERVOMIN)
            time.sleep(1)
            self.set_servo(i, self.SERVOMAX)
            time.sleep(1)
            self.reset_servos()
        messagebox.showinfo("Info", "Servos auto-calibrated.")

    def run_position_loop(self):
        loop_count = int(self.loop_count_entry.get())
        for _ in range(loop_count):
            self.execute_positions()

    def on_closing(self):
        if self.ser:
            self.ser.close()
        self.root.destroy()

    def create_sensors(self):
        self.load_sensor = tk.DoubleVar(value=0)
        self.force_sensor = tk.DoubleVar(value=0)
        # For simulation purposes, update the sensor values every second
        self.update_sensors()

    def update_sensors(self):
        # Simulated sensor values
        self.load_sensor.set(np.random.uniform(0, 10))
        self.force_sensor.set(np.random.uniform(0, 100))
        self.root.after(1000, self.update_sensors)

    def setup_gui(self):
        # Main layout
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill='both', expand=True)
        
        # Side menu
        menu_frame = ttk.Frame(main_frame, padding="10", width=200)
        menu_frame.pack(side='left', fill='y')
        
        # Content frame
        content_frame = ttk.Frame(main_frame, padding="10")
        content_frame.pack(side='right', fill='both', expand=True)
        
        # Menu buttons
        menu_buttons = [
            ("Home", lambda: notebook.select(home_frame)),
            ("Control Panel", lambda: notebook.select(control_frame)),
            ("Speed Control", lambda: notebook.select(speed_frame)),
            ("Graph", lambda: notebook.select(graph_frame)),
            ("Servo Status", lambda: notebook.select(status_frame)),
            ("Movement History", lambda: notebook.select(history_frame)),
            ("Macros", lambda: notebook.select(macro_frame)),
            ("Simulation", lambda: notebook.select(simulation_frame)),
            ("Movement Mode", lambda: notebook.select(mode_frame)),
            ("Loop Control", lambda: notebook.select(loop_frame)),
            ("Monitoring", lambda: notebook.select(monitoring_frame)),
        ]
        
        for text, command in menu_buttons:
            button = ttk.Button(menu_frame, text=text, command=command)
            button.pack(fill='x', pady=5)
        
        # Notebook for content
        notebook = ttk.Notebook(content_frame)
        notebook.pack(fill='both', expand=True)
        
        # Home Frame
        home_frame = ttk.Frame(notebook, padding="10")
        notebook.add(home_frame, text="Home")

        home_title = ttk.Label(home_frame, text="Welcome to Robot Arm Controller", font=('Arial', 20, 'bold'))
        home_title.pack(pady=20)

        # Control frame
        control_frame = ttk.Frame(notebook, padding="10")
        notebook.add(control_frame, text="Control")

        control_title = ttk.Label(control_frame, text="Servo Control", font=('Arial', 14, 'bold'))
        control_title.grid(row=0, column=0, columnspan=2, pady=10)

        # Creating sliders for servo control
        for i in range(6):
            frame = ttk.Frame(control_frame, padding="5")
            frame.grid(row=i+1, column=0, padx=10, pady=5, sticky='ew')
            label = ttk.Label(frame, text=f"Servo {i}")
            label.pack(side="left")
            slider = ttk.Scale(frame, from_=self.SERVOMIN, to_=self.SERVOMAX, orient="horizontal")
            slider.pack(side="left", fill='x', expand=True, padx=5)
            slider.set((self.SERVOMAX - self.SERVOMIN) // 2)  # Initialize to midpoint
            slider.bind("<ButtonRelease-1>", lambda event, s=slider, idx=i: self.on_slider_change(event, s, idx))
            self.sliders.append(slider)

        # Control buttons
        button_frame = ttk.Frame(control_frame, padding="10")
        button_frame.grid(row=7, column=0, padx=10, pady=10, columnspan=2, sticky='ew')

        reset_button = ttk.Button(button_frame, text="Reset Servos", command=self.reset_servos)
        reset_button.grid(row=0, column=0, padx=10, pady=5)

        preset1_button = ttk.Button(button_frame, text="Preset 1", command=lambda: self.preset_position([self.SERVOMIN, self.SERVOMAX, (self.SERVOMAX - self.SERVOMIN) // 2, self.SERVOMAX, self.SERVOMIN, (self.SERVOMAX - self.SERVOMIN) // 2]))
        preset1_button.grid(row=0, column=1, padx=10, pady=5)

        preset2_button = ttk.Button(button_frame, text="Preset 2", command=lambda: self.preset_position([(self.SERVOMAX - self.SERVOMIN) // 2, self.SERVOMAX, self.SERVOMIN, (self.SERVOMAX - self.SERVOMIN) // 2, self.SERVOMAX, self.SERVOMIN]))
        preset2_button.grid(row=0, column=2, padx=10, pady=5)

        save_button = ttk.Button(button_frame, text="Save Position", command=self.save_position)
        save_button.grid(row=1, column=0, padx=10, pady=5)

        execute_button = ttk.Button(button_frame, text="Execute Positions", command=self.execute_positions)
        execute_button.grid(row=1, column=1, padx=10, pady=5)

        save_file_button = ttk.Button(button_frame, text="Save to File", command=self.save_positions_to_file)
        save_file_button.grid(row=1, column=2, padx=10, pady=5)

        load_file_button = ttk.Button(button_frame, text="Load from File", command=self.load_positions_from_file)
        load_file_button.grid(row=1, column=3, padx=10, pady=5)

        # Listbox for saved positions
        list_frame = ttk.Frame(control_frame, padding="10")
        list_frame.grid(row=8, column=0, padx=10, pady=10, columnspan=2, sticky='ew')
        self.saved_positions_list = tk.Listbox(list_frame, height=6)
        self.saved_positions_list.pack(fill='both', expand=True)

        self.saved_positions_list.bind("<<ListboxSelect>>", self.on_position_select)

        # Note entry
        note_frame = ttk.Frame(control_frame, padding="10")
        note_frame.grid(row=9, column=0, columnspan=2, pady=5)
        ttk.Label(note_frame, text="Note:").pack(side='left')
        self.note_entry = ttk.Entry(note_frame)
        self.note_entry.pack(side='left', fill='x', expand=True)
        ttk.Button(note_frame, text="Save Note", command=self.save_note_for_position).pack(side='left', padx=5)

        # Speed control frame
        speed_frame = ttk.Frame(notebook, padding="10")
        notebook.add(speed_frame, text="Speed Control")

        speed_title = ttk.Label(speed_frame, text="Movement Speed", font=('Arial', 14, 'bold'))
        speed_title.grid(row=0, column=0, columnspan=2, pady=10)

        speed_label = ttk.Label(speed_frame, text="Slow")
        speed_label.grid(row=1, column=0, padx=10, pady=10)
        self.speed_slider = ttk.Scale(speed_frame, from_=1, to_=20, orient="horizontal")
        self.speed_slider.grid(row=1, column=1, padx=10, pady=10, sticky='ew')
        self.speed_slider.set(10)  # Initialize to midpoint
        speed_label_fast = ttk.Label(speed_frame, text="Fast")
        speed_label_fast.grid(row=1, column=2, padx=10, pady=10)

        # Graph frame
        graph_frame = ttk.Frame(notebook, padding="10")
        notebook.add(graph_frame, text="Graph")

        graph_title = ttk.Label(graph_frame, text="Servo Positions", font=('Arial', 14, 'bold'))
        graph_title.pack(pady=10)

        self.figure, self.ax = plt.subplots()
        self.graph_canvas = FigureCanvasTkAgg(self.figure, master=graph_frame)
        self.graph_canvas.get_tk_widget().pack(fill='both', expand=True)

        # Servo status frame
        status_frame = ttk.Frame(notebook, padding="10")
        notebook.add(status_frame, text="Servo Status")

        status_title = ttk.Label(status_frame, text="Servo Status", font=('Arial', 14, 'bold'))
        status_title.grid(row=0, column=0, columnspan=2, pady=10)

        self.status_labels = []
        for i in range(6):
            label = ttk.Label(status_frame, text=f"Servo {i}: 0")
            label.grid(row=i+1, column=0, padx=10, pady=5, sticky='w')
            self.status_labels.append(label)

        # Movement history frame
        history_frame = ttk.Frame(notebook, padding="10")
        notebook.add(history_frame, text="Movement History")

        history_title = ttk.Label(history_frame, text="Movement History", font=('Arial', 14, 'bold'))
        history_title.pack(pady=10)

        self.movement_history_list = tk.Listbox(history_frame, height=10)
        self.movement_history_list.pack(fill='both', expand=True, padx=10, pady=10)

        # Macro frame
        macro_frame = ttk.Frame(notebook, padding="10")
        notebook.add(macro_frame, text="Macros")

        macro_title = ttk.Label(macro_frame, text="Macros", font=('Arial', 14, 'bold'))
        macro_title.pack(pady=10)

        macro_control_frame = ttk.Frame(macro_frame, padding="10")
        macro_control_frame.pack(fill='x')

        self.macro_name_entry = ttk.Entry(macro_control_frame)
        self.macro_name_entry.pack(side='left', padx=5)

        save_macro_button = ttk.Button(macro_control_frame, text="Save Macro", command=self.save_macro)
        save_macro_button.pack(side='left', padx=5)

        self.macro_listbox = tk.Listbox(macro_frame, height=10)
        self.macro_listbox.pack(fill='both', expand=True, padx=10, pady=10)
        self.macro_listbox.bind("<<ListboxSelect>>", lambda event: self.load_macro(self.macro_listbox.get(self.macro_listbox.curselection())))

        # Simulation frame
        simulation_frame = ttk.Frame(notebook, padding="10")
        notebook.add(simulation_frame, text="Simulation")

        simulation_title = ttk.Label(simulation_frame, text="Movement Simulation", font=('Arial', 14, 'bold'))
        simulation_title.pack(pady=10)

        self.simulation_canvas = FigureCanvasTkAgg(plt.figure(), master=simulation_frame)
        self.simulation_canvas.get_tk_widget().pack(fill='both', expand=True)
        self.ax_sim = self.simulation_canvas.figure.add_subplot(111, projection='3d')

        # Movement mode frame
        mode_frame = ttk.Frame(notebook, padding="10")
        notebook.add(mode_frame, text="Movement Mode")

        mode_title = ttk.Label(mode_frame, text="Select Movement Mode", font=('Arial', 14, 'bold'))
        mode_title.pack(pady=10)

        linear_radio = ttk.Radiobutton(mode_frame, text="Linear", variable=self.movement_mode, value="Linear")
        linear_radio.pack(side='left', padx=5)
        circular_radio = ttk.Radiobutton(mode_frame, text="Circular", variable=self.movement_mode, value="Circular")
        circular_radio.pack(side='left', padx=5)

        # Loop control frame
        loop_frame = ttk.Frame(notebook, padding="10")
        notebook.add(loop_frame, text="Loop Control")

        loop_title = ttk.Label(loop_frame, text="Position Loop Control", font=('Arial', 14, 'bold'))
        loop_title.pack(pady=10)

        loop_control_frame = ttk.Frame(loop_frame, padding="10")
        loop_control_frame.pack(fill='x')

        ttk.Label(loop_control_frame, text="Loop Count:").pack(side='left', padx=5)
        self.loop_count_entry = ttk.Entry(loop_control_frame)
        self.loop_count_entry.pack(side='left', padx=5)
        ttk.Button(loop_control_frame, text="Run Loop", command=self.run_position_loop).pack(side='left', padx=5)
        
        # Real-time Monitoring frame
        monitoring_frame = ttk.Frame(notebook, padding="10")
        notebook.add(monitoring_frame, text="Monitoring")

        monitoring_title = ttk.Label(monitoring_frame, text="Real-time Monitoring", font=('Arial', 14, 'bold'))
        monitoring_title.pack(pady=10)

        load_label = ttk.Label(monitoring_frame, text="Load Sensor:")
        load_label.pack(pady=5)
        load_value = ttk.Label(monitoring_frame, textvariable=self.load_sensor)
        load_value.pack(pady=5)

        force_label = ttk.Label(monitoring_frame, text="Force Sensor:")
        force_label.pack(pady=5)
        force_value = ttk.Label(monitoring_frame, textvariable=self.force_sensor)
        force_value.pack(pady=5)

    def on_position_select(self, event):
        selection = event.widget.curselection()
        if selection:
            index = selection[0]
            self.go_to_position(index)

    def update_graph(self):
        positions = [slider.get() for slider in self.sliders]
        self.ax.clear()
        self.ax.bar(range(len(positions)), positions, color='blue')
        self.ax.set_ylim(self.SERVOMIN, self.SERVOMAX)
        self.graph_canvas.draw()

    def update_servo_status(self):
        for i, slider in enumerate(self.sliders):
            self.status_labels[i].config(text=f"Servo {i}: {int(slider.get())}")

    def update_movement_history_list(self):
        self.movement_history_list.delete(0, tk.END)
        for idx, pos in enumerate(self.movement_history):
            self.movement_history_list.insert(tk.END, f"Move {idx+1}: {pos}")

    def update_simulation(self):
        self.ax_sim.clear()
        positions = [slider.get() for slider in self.sliders]
        
        # Simple 3D simulation of a robotic arm
        base = [0, 0, 0]
        links = [
            np.array([0, 0, 1]),
            np.array([1, 0, 1]),
            np.array([2, 0, 1]),
            np.array([2, 1, 1]),
            np.array([2, 2, 1]),
            np.array([2, 2, 2])
        ]
        
        x, y, z = [base[0]], [base[1]], [base[2]]
        
        for i in range(len(links)):
            x.append(links[i][0])
            y.append(links[i][1])
            z.append(links[i][2])
        
        self.ax_sim.plot(x, y, z, marker='o')
        self.ax_sim.set_xlim([-3, 3])
        self.ax_sim.set_ylim([-3, 3])
        self.ax_sim.set_zlim([0, 3])
        self.simulation_canvas.draw()

if __name__ == "__main__":
    controller = RobotArmController('/dev/cu.usbmodem21201', 115200)
    controller.root.mainloop()
