import tkinter as tk
from tkinter import ttk


class ControlGUI:
    def __init__(self, camera_stream):
        self.camera_stream = camera_stream
        self.root = tk.Tk()
        self.root.title("Camera Control Panel")

        self.slice_width_var = tk.IntVar(value=25)

        slice_width_label = tk.Label(self.root, text="Slice Width (%)")
        slice_width_label.pack(pady=10)

        slice_width_slider = tk.Scale(
            self.root,
            from_=1,
            to=100,
            orient=tk.HORIZONTAL,
            variable=self.slice_width_var,
            command=self.on_slice_width_change,
        )
        slice_width_slider.pack(pady=10)

        fullscreen_button = tk.Button(
            self.root, text="Toggle Fullscreen (f)", command=self.toggle_fullscreen
        )
        fullscreen_button.pack(pady=10)

        x_freeze_button = tk.Button(
            self.root, text="Toggle X Freeze (x)", command=self.toggle_x_freeze
        )
        x_freeze_button.pack(pady=10)

        y_freeze_button = tk.Button(
            self.root, text="Toggle Y Freeze (y)", command=self.toggle_y_freeze
        )
        y_freeze_button.pack(pady=10)

        z_freeze_button = tk.Button(
            self.root, text="Toggle Z Freeze (z)", command=self.toggle_z_freeze
        )
        z_freeze_button.pack(pady=10)

        calibration_button = tk.Button(
            self.root, text="Toggle Calibration (c)", command=self.toggle_calibration
        )
        calibration_button.pack(pady=10)

        set_calibration_button = tk.Button(
            self.root, text="Set Calibration (v)", command=self.set_current_calibration
        )
        set_calibration_button.pack(pady=10)

        reset_alignment_button = tk.Button(
            self.root, text="Reset Alignment (r)", command=self.reset_alignment
        )
        reset_alignment_button.pack(pady=10)

        stop_button = tk.Button(self.root, text="Stop Stream", command=self.stop_stream)
        stop_button.pack(pady=10)

        self.is_fullscreen = False

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def on_slice_width_change(self, val):
        self.camera_stream.slice_width_percent = int(val)

    def toggle_fullscreen(self):
        self.is_fullscreen = not self.is_fullscreen
        self.camera_stream.set_fullscreen(self.is_fullscreen)

    def toggle_x_freeze(self):
        self.camera_stream.x_freeze = not self.camera_stream.x_freeze
        print(f"X Freeze: {self.camera_stream.x_freeze}")

    def toggle_y_freeze(self):
        self.camera_stream.y_freeze = not self.camera_stream.y_freeze
        print(f"Y Freeze: {self.camera_stream.y_freeze}")

    def toggle_z_freeze(self):
        self.camera_stream.z_freeze = not self.camera_stream.z_freeze
        print(f"Z Freeze: {self.camera_stream.z_freeze}")

    def toggle_calibration(self):
        self.camera_stream.calibration_active = (
            not self.camera_stream.calibration_active
        )
        print(f"Calibration mode: {self.camera_stream.calibration_active}")

    def set_current_calibration(self):
        self.camera_stream._set_min_max_to_current(
            (
                self.camera_stream.alignment_x,
                self.camera_stream.alignment_y,
                self.camera_stream.alignment_z,
            )
        )
        print("Calibration values set to current alignment")

    def reset_alignment(self):
        self.camera_stream._set_zero_alignment(
            self.camera_stream.current_alignment_angle
        )

    def stop_stream(self):
        self.camera_stream.stop_stream()

    def run(self):
        """Run the Tkinter event loop in the main thread."""
        self.root.mainloop()

    def on_closing(self):
        """Handle the Tkinter window close event."""
        print("Closing the application...")
        self.stop_stream()
        self.root.destroy()
