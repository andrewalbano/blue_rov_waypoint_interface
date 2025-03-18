# import tkinter as tk

# root = tk.Tk()
# root.title("Scrollable Tkinter Window")

# # Create a Canvas widget with scrollbars
# canvas = tk.Canvas(root, borderwidth=0)
# vertical_scrollbar = tk.Scrollbar(root, orient="vertical", command=canvas.yview)
# canvas.configure(yscrollcommand=vertical_scrollbar.set)

# # Place the scrollbar and canvas
# vertical_scrollbar.pack(side="right", fill="y")
# canvas.pack(side="left", fill="both", expand=True)

# # Create a frame inside the canvas
# scrolling_frame = tk.Frame(canvas)

# # Add the frame to the canvas
# canvas.create_window((0, 0), window=scrolling_frame, anchor="nw")

# # Function to update the scroll region
# def configure_scroll_region(event):
#     canvas.configure(scrollregion=canvas.bbox("all"))

# # Bind the frame's configure event to update the scroll region
# scrolling_frame.bind("<Configure>", configure_scroll_region)

# # Add content to the scrolling frame
# for i in range(50):
#     tk.Label(scrolling_frame, text=f"Label {i}").pack()
#     tk.Button(scrolling_frame, text=f"Button {i}").pack()

# root.mainloop()
import tkinter as tk
from tkinter import Frame, LabelFrame, Canvas, Scrollbar

class WaypointGui:
    def __init__(self, master):
        # Initialize variables (not fully listed for brevity)
        self.master = master
        self.master.title("Manage Waypoints")

        # Create the main container
        self.main_frame = Frame(master)
        self.main_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # Create the outer frame
        self.outer_frame = Frame(self.main_frame)
        self.outer_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # Create a canvas and add it to the outer frame
        self.canvas = Canvas(self.outer_frame)
        self.canvas.pack(side="left", fill="both", expand=True)

        # Create a scrollbar and attach it to the outer frame
        self.scrollbar = Scrollbar(self.outer_frame, orient="vertical", command=self.canvas.yview)
        self.scrollbar.pack(side="right", fill="y")

        # Configure the canvas to work with the scrollbar
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        self.canvas.bind('<Configure>', lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))

        # Create the inner frame and attach it to the canvas
        self.inner_frame = Frame(self.canvas)
        self.canvas.create_window((0, 0), window=self.inner_frame, anchor="nw")

        # Add widgets to the inner frame
        self.waypoints_frame = LabelFrame(self.inner_frame, text="Add Waypoints")
        self.waypoints_frame.grid(row=0, column=0, columnspan=2, pady=10, sticky="ew")

        # Further widget configuration in the inner frame (examples)
        self.add_waypoint_frame_1 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nNED frame")
        self.add_waypoint_frame_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")

        self.add_waypoint_frame_2 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe last waypoint added")
        self.add_waypoint_frame_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")

        self.add_waypoint_frame_3 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe current pose")
        self.add_waypoint_frame_3.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")

        self.position_pid_gains = LabelFrame(self.waypoints_frame, text="Position PID Gains")
        self.position_pid_gains.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")

        self.params = LabelFrame(self.waypoints_frame, text="Other Params")
        self.params.grid(row=1, column=2, columnspan=2, padx=5, pady=5, sticky="nsew")

        self.PWM_pid_gains = LabelFrame(self.waypoints_frame, text="PWM PID Gains")
        self.PWM_pid_gains.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="nsew")

        self.velocity_testing = LabelFrame(self.waypoints_frame, text="Velocity Setpoint Testing")
        self.velocity_testing.grid(row=2, column=2, columnspan=2, padx=5, pady=5, sticky="nsew")
        
        # Make sure Widgets adjust as the window resizes
        for i in range(3):
            self.inner_frame.columnconfigure(i, weight=1)
        for i in range(3):
            self.inner_frame.rowconfigure(i, weight=1)

# To run the application
if __name__ == "__main__":
    root = tk.Tk()
    gui = WaypointGui(root)
    root.mainloop()