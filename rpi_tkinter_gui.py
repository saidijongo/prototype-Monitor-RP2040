
import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
from PIL import ImageTk, Image
import serial
from multiprocessing import Process, Queue
import time


class ArduinoController:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.ser = None

    def connect(self):
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            return True
        except serial.SerialException:
            return False

    def disconnect(self):
        if self.ser:
            self.ser.close()

    def send_command(self, command):
        if self.ser and self.ser.is_open:
            self.ser.write((command + '\n').encode())
            return True
        else:
            return False


class ArduinoGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Control GUI")

        self.arduino_controller = ArduinoController()

        self.create_gui()

    def create_gui(self):
        self.connection_label = tk.Label(self.root, text="Arduino Connection: Not Connected", font=("Helvetica", 12))
        self.connection_label.pack(pady=10)

        self.connect_button = tk.Button(self.root, text="Connect Arduino", command=self.connect_arduino)
        self.connect_button.pack(pady=10)

        # Create buttons for each device
        devices = ["PRINTER", "CARD_DISPENSER", "CARD_READER", "PASSPORT_READER", "MOTOR", "HOME"]
        for device in devices:
            frame = ttk.Frame(self.root)
            frame.pack(pady=10)
            ttk.Button(frame, text=f"{device} ON", command=lambda d=device: self.send_command(f"ST,0,{d},1,ED")).pack(side=tk.LEFT, padx=5)
            ttk.Button(frame, text=f"{device} OFF", command=lambda d=device: self.send_command(f"ST,0,{d},0,ED")).pack(side=tk.LEFT, padx=5)

    def connect_arduino(self):
        if self.arduino_controller.connect():
            self.connection_label.config(text="Arduino Connection: Connected", fg="green")
            self.connect_button.config(state=tk.DISABLED)
        else:
            messagebox.showerror("Connection Error", "Failed to connect to Arduino")

    def send_command(self, command):
        if self.arduino_controller.send_command(command):
            print(f"Sent command: {command}")
        else:
            messagebox.showerror("Connection Error", "Arduino not connected. Please connect first.")


if __name__ == "__main__":
    root = tk.Tk()
    gui = ArduinoGUI(root)
    root.mainloop()
