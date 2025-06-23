import math
import threading
import tkinter as tk
from tkinter import ttk


class VirtualJoystick(tk.Frame):
    def __init__(self, parent, size=200, **kwargs):
        super().__init__(parent, **kwargs)
        self.size = size
        self.center = size // 2
        self.max_distance = size // 2 - 10

        self.canvas = tk.Canvas(self, width=size, height=size, bg="lightgray")
        self.canvas.pack(padx=10, pady=10)

        self.canvas.create_oval(5, 5, size - 5, size - 5, outline="black", width=2)

        self.knob_size = 20
        self.knob_x = self.center
        self.knob_y = self.center
        self.knob = self.canvas.create_oval(
            self.knob_x - self.knob_size // 2,
            self.knob_y - self.knob_size // 2,
            self.knob_x + self.knob_size // 2,
            self.knob_y + self.knob_size // 2,
            fill="red",
            outline="darkred",
            width=2,
        )

        self.canvas.bind("<Button-1>", self.on_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<ButtonRelease-1>", self.on_release)

        self.command_callback = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

    def set_command_callback(self, callback):
        """Set callback function to be called when joystick position changes"""
        self.command_callback = callback

    def on_click(self, event):
        self.move_knob(event.x, event.y)

    def on_drag(self, event):
        self.move_knob(event.x, event.y)

    def on_release(self, event):
        # Return to center
        self.move_knob(self.center, self.center)

    def move_knob(self, x, y):
        dx = x - self.center
        dy = y - self.center
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > self.max_distance:
            angle = math.atan2(dy, dx)
            x = self.center + self.max_distance * math.cos(angle)
            y = self.center + self.max_distance * math.sin(angle)
            dx = x - self.center
            dy = y - self.center
            distance = self.max_distance

        self.knob_x = x
        self.knob_y = y
        self.canvas.coords(
            self.knob,
            x - self.knob_size // 2,
            y - self.knob_size // 2,
            x + self.knob_size // 2,
            y + self.knob_size // 2,
        )

        self.current_x = dx / self.max_distance
        self.current_y = -dy / self.max_distance
        self.current_z = 0.0

        if self.command_callback:
            self.command_callback(self.current_x, self.current_y, self.current_z)


class UIController:
    def __init__(self, stand_callback = lambda: None, sit_callback = lambda: None, bob_callback = lambda: None):
        self.stand_callback = stand_callback
        self.sit_callback = sit_callback
        self.bob_callback = bob_callback

        self.root = tk.Tk()
        self.root.title("Virtual Joystick")
        self.root.resizable(False, False)  # Prevent resizing
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack()

        joystick_frame = ttk.Frame(main_frame)
        joystick_frame.pack()

        left_frame = ttk.Frame(joystick_frame)
        left_frame.grid(row=0, column=0, padx=15)

        left_title = ttk.Label(left_frame, text="X+Y Velocity", font=("Arial", 12, "bold"))
        left_title.pack(pady=(0, 5))

        self.left_joystick = VirtualJoystick(left_frame, size=160)
        self.left_joystick.pack()
        self.left_joystick.set_command_callback(self.update_xy_velocity)

        right_frame = ttk.Frame(joystick_frame)
        right_frame.grid(row=0, column=1, padx=15)

        right_title = ttk.Label(right_frame, text="X+Yaw Velocity", font=("Arial", 12, "bold"))
        right_title.pack(pady=(0, 5))

        self.right_joystick = VirtualJoystick(right_frame, size=160)
        self.right_joystick.pack()
        self.right_joystick.set_command_callback(self.update_x_yaw_velocity)

        # Add buttons frame
        buttons_frame = ttk.Frame(main_frame)
        buttons_frame.pack(pady=10)

        # Create and pack the buttons
        self.button1 = ttk.Button(buttons_frame, text="Stand", command=self.stand_callback)
        self.button1.grid(row=0, column=0, padx=10)

        self.button2 = ttk.Button(buttons_frame, text="Sit", command=self.sit_callback)
        self.button2.grid(row=0, column=1, padx=10)

        self.button3 = ttk.Button(buttons_frame, text="Bob", command=self.bob_callback)
        self.button3.grid(row=0, column=2, padx=10)

        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0

        self.linear_scale = 2.0  # m/s
        self.angular_scale = 10.0  # rad/s

        self.root.update_idletasks()
        self.root.after(100, self.update_gui)

    def update_xy_velocity(self, x, y, z):
        """Handle X+Y velocity from left joystick"""
        self.linear_x = y * self.linear_scale
        self.linear_y = -x * self.linear_scale

    def update_x_yaw_velocity(self, x, y, z):
        """Handle X+Yaw velocity from right joystick"""
        self.linear_x = y * self.linear_scale
        self.angular_z = -x * self.angular_scale / 4.0

    def get_state(self):
        """Return current joystick state"""
        return self.linear_x, self.linear_y, self.angular_z

    def update_gui(self):
        """Update GUI periodically"""
        self.root.after(50, self.update_gui)

    def on_closing(self):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.root.quit()

    def run(self):
        self.root.mainloop()


def main():
    try:
        ui_controller = UIController()
        ui_controller.run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
