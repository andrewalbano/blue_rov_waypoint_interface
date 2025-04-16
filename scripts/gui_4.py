#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped,PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, euler_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, euler_from_quaternion, euler_from_matrix
# from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox, Frame, Toplevel
import tkinter as tk
from tkinter import *
import tkinter.ttk as ttk
from tkinter.ttk import *
from std_msgs.msg import Bool,Int8,Float32MultiArray, String, Int8MultiArray, MultiArrayDimension
from nav_msgs.msg import Path
import utm
import tf
# import dubins

# need to find a way to get the initial coordinates that the ned frame is located at

class windows(Tk):
    def __init__(self, *args, **kwargs): 
        Tk.__init__(self, *args, **kwargs)
        # self.br = tf.TransformBroadcaster()  
        self.pub1 = rospy.Publisher('new_pose_visualization', PoseStamped, queue_size=10)
        self.pub3 = rospy.Publisher("waypoint_plot_visualization", PoseArray, queue_size=10)
        self.pub4 = rospy.Publisher("target_waypoints_list", PoseArray, queue_size=10) # not used yet
        self.pub10 = rospy.Publisher("motion_controller_state",String, queue_size=1)
        self.pub5 = rospy.Publisher("desired_robot_path", Path, queue_size=10)
        self.pub7 = rospy.Publisher("hold_pose", Bool, queue_size=1)
        self.pub9 = rospy.Publisher("controller_gains", Float32MultiArray, queue_size=10)
        self.pub10 = rospy.Publisher("motion_controller_state",String, queue_size=1)
        self.pub12 = rospy.Publisher("add_waypoint", PoseStamped, queue_size=10)
        self.pub13 = rospy.Publisher("reset_waypoints", Bool, queue_size=10)
        self.pub14 = rospy.Publisher("path_generation_type", Int8MultiArray, queue_size=10)
        self.pub15 = rospy.Publisher("path_orientation_style", Int8MultiArray, queue_size=10)
        self.pub16 = rospy.Publisher("visualize_preset_pattern", PoseArray, queue_size=1) # not used yet
        self.pub17= rospy.Publisher("path_planner_parameters", Float32MultiArray, queue_size=10) # not used yet
        self.pub18 = rospy.Publisher("visualize_preset_pattern_path", Path, queue_size=1) # not used yet
        # self.pub19 = rospy.Publisher("state", PoseWithCovarianceStamped, queue_size=1) # not used yet
        # Setting window icon
        self.icon = PhotoImage(file="/home/andrew/bluerov_waypoint_follower/src/blue_rov_waypoint_interface/scripts/desktop_image_2.png")
        self.iconphoto(True, self.icon)


        # Sets the current mode of the controller
        # controller modes are Disabled, waypoint, velocity, manual pwm, Joystick
        self.current_mode = "Disabled"

        # Updates the text in the activate/deactivate button
        self.controller_active = False
        # when hold pose is true when waypoint mode is initiated
        self.hold_pose = True

        # state of the popup windows
        self.open_controller_params_frame = False
        self.open_preset_patterns_frame = False
        # need to initialize the names of the windows 
        self.controller_params_window = None
        self.preset_patterns_window = None

        # initialize target depth
        self.target_depth = None

        # initial lat,long, alt
        self.init_lat_ref = 42.2808
        self.init_lon_ref = -83.7430

        self.init_x_utm , self.init_y_utm, _, _ = utm.from_latlon(self.init_lat_ref, self.init_lon_ref, force_northern=True)
        

        # Initialize variables for controller gains
        self.kp_x = 0.0
        self.kd_x = 0.0
        self.ki_x = 0.0

        self.kp_y = 0.0
        self.kd_y = 0.0
        self.ki_y = 0.0

        self.kp_z = 0.0
        self.kd_z = 0.0
        self.ki_z = 0.0

        self.kp_yaw = 0.0
        self.kd_yaw = 0.0
        self.ki_yaw = 0.0

        # the first value in data indicates what the rest of the data will be to the subscriber in motion controller
        self.motion_controller_msgs  = Float32MultiArray() 
        self.motion_controller_msgs.data = []   

        # the first value in data indicates what the rest of the data will be to the subscriber in motion controller
        self.path_planner_msgs  = Float32MultiArray() 
        self.path_planner_msgs.data = []   

        # for preset pattern visualization
        self.preset_pattern = PoseArray()
        self.preset_pattern.header.frame_id="NED"

        self.preset_pattern_path = Path()
        self.preset_pattern_path.header.frame_id = "NED"

        self.temp_pose = PoseStamped()



    
    
        # we dont really need to call back the values
        # used to set limits for safe operation
        # self.max_linear_velocity = 0
        # self.min_pwm = 0
        # self.max_pwm = 0
    
        # used in velocity mode for velocity setpoints
        # self.vx = 0
        # self.vy = 0
        # self.vz = 0
        # self.vyaw = 0

        # this stores the last waypoint visualized relative to the current state at the time of visualizing
        #  when you add a waypoint relative to current state, you need to visualize then add it, this is a safety feature to prevent issues 
        self.last_waypoint_rel_to_current_state_visualized = Pose()
        self.last_waypoint_rel_to_current_state_visualized = None
        #  this holds the transform for the last waypoint visualized
        self.last_waypoint_rel_to_current_state_visualized_transform =np.eye(4)

        #  this is used to show the straight line path between waypoints
        self.desired_path = Path()
        self.desired_path.header.frame_id='NED'


        # For each waypoint that is published the orientation style used to go between the previous waypoint and the last added waypoint is 
        self.orientation_style = Int8MultiArray()
        self.orientation_style.layout.dim=[]
        self.orientation_style.data = []


        # stores a list of the target waypoints
        self.goal_waypoints = PoseArray()
        self.goal_waypoints.header.frame_id = 'NED'

        # stores the transformation for the last waypoint 
        self.last_waypoint_transform = np.eye(4)
        self.last_waypoint = Pose()
    
        # stores the current pose
        self.current_pose = PoseWithCovarianceStamped()
        # self.init_pose = PoseStamped()
        # self.init_pose.header.frame_id = self.current_pose.header.frame_id
        # self.init_pose.pose= self.current_pose.pose.pose
   
        # Adding a title to the window
        self.wm_title("BlueROV2 Heavy Configuration Interface")

        # creating a frame and assigning it to container
        self.main_container = Frame(self)
        # self.main_container.grid(row=0,column=0,padx=10,pady=10, sticky="nsew")
        self.main_container.grid(row=0,column=0,sticky="nsew")
        

        # configuring the location of the container using grid
        self.main_container.grid_rowconfigure(0, weight=1)
        self.main_container.grid_columnconfigure(0, weight=1)

        # Status Frame
        #  prints the current mode of the controller
        self.statusFrame = LabelFrame(self.main_container,text = "Status")
        self.statusFrame.grid(row=0, column=0, padx=10,pady=10, sticky="w")

        self.current_mode_label = Label(self.statusFrame, text="Current Mode: Disabled")
        self.current_mode_label.grid(row=1, column=0, padx=20, pady=5,sticky="w")

        self.depth_frame = LabelFrame(self.main_container,text = "Depth Information")
        self.depth_frame.grid(row=1, column=0, padx=10,pady=10, sticky="w")

        #  depth info frame
        self.current_z_label = Label(self.depth_frame,text= "Current Depth:")
        self.current_z_label.configure(text=f"Current Depth: {round(self.current_pose.pose.pose.position.z,2)} m")
        self.current_z_label.grid(row=2, column=0, columnspan=3, padx=20, pady=5,sticky="w")

        self.target_depth_label = Label(self.depth_frame,text= "Target Depth")
        self.target_depth_label.configure(text=f"Target Depth: Not Set", state = HIDDEN)
        self.target_depth_label.grid(row=3, column=0,columnspan=3, padx=20, pady=5,sticky="w")


        # depth setpoint frame
        self.depth_setpoint_frame = LabelFrame(self.depth_frame, text="Set Operating Depth")
        self.depth_setpoint_frame.grid(row = 4, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        depth = StringVar()
        depth_entry = Entry(self.depth_setpoint_frame,textvariable = depth, width=10).grid(row=4, column=0,padx=5, pady=5, sticky="w")
        self.target_depth_button = tk.Button(self.depth_setpoint_frame,text = "Submit", state=DISABLED, command=lambda: self.submit_target_depth(depth.get()))
        self.target_depth_button.grid(row=4, column=1, padx=10,pady=10, sticky="w")


        # on off frame
        on_off_frame = LabelFrame(self.main_container,text = "On / Off")
        on_off_frame.grid(row=2, column=0, padx=10,pady=10, sticky="w")

        self.on_off_button = Button(
            on_off_frame,
            text="Activate Controller",
            command=self.on_off_button_function
        )
        self.on_off_button.grid(row=0, column=0, columnspan=1, padx=20, pady=20,sticky="ew")
        
        # a dictionary of frames
        self.frames = {}
        for F in (InfoPage,WaypointFrame,ControlOptionsPage,JoystickFrame,VelocityFrame,PWMFrame):
            frame = F(self.main_container, self)

            # the windows class acts as the root window for the frames.
            self.frames[F] = frame
                    
        
        # need to reset after creating the other frames above
        #  Current mode of the controller
        self.current_mode = "Disabled"

        # Updates the text in the activate/deactivate button
        self.controller_active = False

        # Displaying the current mode and on off button text
        self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
        self.on_off_button.configure(text="Activate Controller")

        # show th info frame 
        self.show_frame(InfoPage)

        self.sub1 = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        self.sub2 = rospy.Subscriber('/holdpose', PoseWithCovarianceStamped, self.position_callback)

    def open_close_preset_patterns_window(self):
        '''     
        Check if the frame is not open:
            Attempt to close the window and change the button text to indicate that the window can be opened. If there's nothing to close, a warning is logged.

        Check if the frame is open:
            If the window exists, try to bring it to the front.
            If it can't be brought to the front (possibly because it was already destroyed), or if it doesn't exist, create a new instance of the window.
        '''

        # Check if the preset patterns frame is not open
        if not self.open_preset_patterns_frame:
            # Try to destroy/close the preset patterns window if it exists
            try:
                self.preset_patterns_window.destroy()
                
                # Update the button text to indicate the window can be opened
                self.frames[WaypointFrame].open_preset_patterns_button.configure(text="Open Preset Patterns")
            
            # Catch any exceptions, most likely that there is no window to close, usually if it was closed using the x button instead of the close window button
            except:
                rospy.logwarn("No window to close")

        # If the preset patterns frame is supposed to be open
        elif self.open_preset_patterns_frame:
            # Check if the window object already exists
            if self.preset_patterns_window:
                # Try to bring the window to the foreground
                try:
                    self.preset_patterns_window.deiconify()
                
                # If deiconifying fails, create a new window instance
                except:
                    self.preset_patterns_window = PresetPatternsFrame(self.main_container, self)
            else:
                # If the window doesn't exist, create a new instance of the preset patterns window
                self.preset_patterns_window = PresetPatternsFrame(self.main_container, self)

    def open_close_controller_params_window(self):
        # Check if the controller parameters frame needs to be closed
        if not self.open_controller_params_frame:
            # Try to destroy/close the controller parameters window if it exists
            try:
                self.controller_params_window.destroy()
                
                # Update the button text in both WaypointFrame and VelocityFrames
                self.frames[WaypointFrame].open_controller_params_button.configure(text="Edit Controller Parameters")
                self.frames[VelocityFrame].open_controller_params_button.configure(text="Edit Controller Parameters")

            # Catch any exceptions, likely that it was aready closed
            except:
                rospy.logwarn("Nothing to close")

        # If the controller parameters frame is supposed to be open
        elif self.open_controller_params_frame:
            # Check if the window object already exists
            if self.controller_params_window:
                # Try to bring the window to the foreground
                try:
                    self.controller_params_window.deiconify()
                
                # If deiconifying fails, create a new window instance
                except:
                    self.controller_params_window = ControllerParamFrame(self.main_container, self)
            else:
                 # If the window doesn't exist, create a new instance of the controller parameters window
                self.controller_params_window = ControllerParamFrame(self.main_container, self)
                
    def show_frame(self, cont):
  
        if cont == ControlOptionsPage:
            self.current_mode = "Initialized"
            self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
            self.target_depth_button.config(state=NORMAL)
            self.target_depth = self.current_pose.pose.pose.position.z
            self.target_depth_label.configure(text=f"Target Depth: {round(self.current_pose.pose.pose.position.z,2)} m")
            

        elif cont == JoystickFrame:
            self.current_mode = "Joystick"
            self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
            self.target_depth_button.config(state=DISABLED)
            self.target_depth = None
            self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
           
            
        elif cont == WaypointFrame:
            self.current_mode = "waypoint"
            self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
            # self.hold_pose = True
            self.frames[cont].hold_pose_button.configure(text="Follow Path")
            self.frames[cont].open_controller_params_button.configure(text="Edit Controller Parameters")
            self.frames[cont].open_preset_patterns_button.configure(text="Open Preset Patterns")

            self.target_depth_button.config(state=DISABLED)
            self.target_depth = None
            self.target_depth_label.configure(text=f"Targeting Waypoint Depth")

            if self.goal_waypoints.poses:
                self.erase_waypoints()
        elif cont == VelocityFrame:
            self.current_mode = "velocity"
            self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
            self.frames[cont].open_controller_params_button.configure(text="Edit Controller Parameters")

            self.target_depth_button.config(state=DISABLED)
            self.target_depth = None
            self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
           
            
            
        elif cont == PWMFrame:
            self.current_mode = "manual pwm"
            self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")

            self.target_depth_button.config(state=DISABLED)
            self.target_depth = None
            self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
           
            

        frame = self.frames[cont]
        frame.grid(row=4, column=0,padx=10,pady=10, sticky="nsew")
        # raises the current frame to the top
        frame.tkraise()
        self.pub10.publish(self.current_mode)

    def position_callback(self, msg):
        # updates the current pose and edits the depth display value
        self.current_pose = msg
        self.current_pose_transform = self.get_current_pose_transform()
        self.current_z_label.configure(text=f"Current Depth: {round(self.current_pose.pose.pose.position.z,2)} m")

    def get_current_pose_transform(self):
        # convert current pose to transformation matrix in NED frame
        trans_matrix = translation_matrix([self.current_pose.pose.pose.position.x,self.current_pose.pose.pose.position.y,self.current_pose.pose.pose.position.z])
        roll, pitch, yaw = euler_from_quaternion([self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w])
        rot_matrix = euler_matrix(roll,pitch,yaw)
        transform  = concatenate_matrices(trans_matrix,rot_matrix)
        return transform
        
    def on_off_button_function(self):
        # completely activate / deactivate the controller 
        # updates the mode, changes button text and brings up the frame to choose which control mode to use, also allows a target depth value to be set
        if not self.controller_active:
            self.current_mode = "Initialized"
            self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
            self.on_off_button.configure(text="Disable Controller")
            self.controller_active = True
            self.show_frame(ControlOptionsPage)
            self.target_depth_button.config(state=NORMAL)
            self.target_depth = self.current_pose.pose.pose.position.z
            self.target_depth_label.configure(text=f"Target Depth: {round(self.current_pose.pose.pose.position.z,2)} m")
                       
        elif self.controller_active:
            self.current_mode = "Disabled"
            self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
            self.on_off_button.configure(text="Activate Controller")
            self.controller_active = False
            self.show_frame(InfoPage)
            self.target_depth_button.config(state=DISABLED)
            self.target_depth = None
            self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
           
            #  if it exists
            if self.controller_params_window:
                self.controller_params_window.destroy()
            
    def set_zero_velocity(self):
        
        indicator = 5

        # # setting velocity to zero
        self.vx_setpoint = 0
        self.vy_setpoint = 0
        self.vz_setpoint = 0
        self.vyaw_setpoint = 0
        
        # preparing information to be passed
        self.motion_controller_msgs.data = [
                    indicator,
                    0,
                    0,
                    0,
                    0
                ]
        
            
        self.pub9.publish(self.motion_controller_msgs)
        rospy.loginfo("Setting velocity to zero")
    
    def submit_velocity_setpoint(self,vx,vy,vz,vyaw):
        # It does not warn you if the value assigned is larger than your max set value 
        # indicator tells the subscriber what information is being passed 
        indicator = 5

        # setting velocity to zero
        self.vx_setpoint = round(self.entry_to_float(vx),3)
        self.vy_setpoint = round(self.entry_to_float(vy),3)
        self.vz_setpoint = round(self.entry_to_float(vz),3)
        self.vyaw_setpoint = round(self.entry_to_float(vyaw),3)
        
        # preparing information to be passed
        self.motion_controller_msgs.data = [
                    indicator,
                    self.vx_setpoint,        
                    self.vy_setpoint,
                    self.vz_setpoint,
                    self.vyaw_setpoint
                ]
        
            
        self.pub9.publish(self.motion_controller_msgs)
        rospy.loginfo("Setting new velocity setpoints")
    
    def submit_target_depth(self,val):

            # indicator tells the subscriber what information is being passed
        indicator = 6
        self.target_depth = self.entry_to_float(val)
        # preparing information to be passed
        self.motion_controller_msgs.data = [
                indicator,
                self.target_depth
            ]
        
        # updating gui
        self.target_depth_label.configure(text=f"Target Depth: {round(self.target_depth,2)} m")

        # publishing information
        self.pub9.publish(self.motion_controller_msgs)
        rospy.loginfo(f"Setting target depth to {val} m")
    
    def set_zero_pwm(self):
        indicator = 7

        # setting PWM to zero
        self.x_pwm = 0
        self.y_pwm = 0
        self.z_pwm = 500 
        self.yaw_pwm  = 0
    
        # preparing information to be passed
        self.motion_controller_msgs.data = [
                indicator,
                0,
                0,
                500,
                0
            ]
            
        self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
        self.pub9.publish(self.motion_controller_msgs)
        rospy.loginfo("Setting pwm to zero")
    
    def submit_pwm_setpoint(self,x_pwm,y_pwm,z_pwm,yaw_pwm):
            
        # indicator tells the subscriber what information is being passed 
        indicator = 7

        # setting PWM to zero
        self.x_pwm = self.entry_to_float(x_pwm)
        self.y_pwm = self.entry_to_float(y_pwm)
        self.z_pwm = self.entry_to_float(z_pwm)
        self.yaw_pwm  = self.entry_to_float(yaw_pwm)
    
        # quick work around for the differing scale and for when z is left blank
        if self.z_pwm == 0.0:
            self.z_pwm  = 500

        if self.z_pwm == 500:
            self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
        else:
            self.target_depth_label.configure(text=f"Target Depth: Manually adjusting depth")

    
        # preparing information to be passed
        self.motion_controller_msgs.data = [
                indicator,
                self.x_pwm,
                self.y_pwm,
                self.z_pwm,
                self.yaw_pwm
            ]

    
        self.pub9.publish(self.motion_controller_msgs)
        rospy.loginfo("Sending PWM setpoints")

    def visualize_waypoint_1(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to current pose"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            # update the transform for the waypoint
            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)
            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            
            # convert relative transform relative to body frame to Ned frame
            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)    
            transform = concatenate_matrices(trans_matrix, rot_matrix)            
            transform = concatenate_matrices(self.get_current_pose_transform(), transform)

            # convert to pose msg format
            translation = translation_from_matrix(transform)            
            quaternion = quaternion_from_matrix(transform)

            # updating pose_msg
            pose_msg.position.x = translation[0]
            pose_msg.position.y = translation[1]
            pose_msg.position.z = translation[2]
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            self.last_waypoint_rel_to_current_state_visualized  = pose_msg
            self.last_waypoint_rel_to_current_state_visualized_transform = transform
        
            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
           

            self.pub1.publish(pose_stamped_msg)


            self.frames[WaypointFrame].submit_waypoint_button_1.config(state=NORMAL)
        

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")


            # messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")

    def visualize_waypoint_2(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to last waypoint added"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'


            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)

            transform = concatenate_matrices(trans_matrix, rot_matrix) #makes translation in current frame then rotates the orientationof the body
            transform = concatenate_matrices(self.last_waypoint_transform, transform)


            # convert to pose msg format
            translation = translation_from_matrix(transform) 
            quaternion = quaternion_from_matrix(transform)
           



            # updating pose_msg
            pose_msg.position.x = translation[0] 
            pose_msg.position.y = translation[1]
            pose_msg.position.z = translation[2]
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]


            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
            self.pub1.publish(pose_stamped_msg)

            self.frames[WaypointFrame].submit_waypoint_button_1.config(state=DISABLED)
            
            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")

    def visualize_waypoint_3(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to NED"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            
        
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]
            

            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
            self.pub1.publish(pose_stamped_msg)

            self.frames[WaypointFrame].submit_waypoint_button_1.config(state=DISABLED)

            
            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")

    def visualize_waypoint_4(self,lat,lon,Z,yaw):
        
        # need to verify this, particularly the depth
        # since the distance is less than 100 m we can assume flat plane tangent
        # yaw is relative to starting yaw
      
        """Visualize the single waypoint relative to gps coord, sea level and starting yaw"""
        try:
            roll = 0
            pitch = 0
           
            lat = self.entry_to_float(lat)
            lon = self.entry_to_float(lon)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            # convert from alt lon to utm then get distance from ned origin UTM coord

            x_utm, y_utm, _, _= utm.from_latlon(lat, lon, force_northern=True)

            x = x_utm - self.init_x_utm
            y = y_utm - self.init_y_utm


            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            
        
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]
            

            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
            self.pub1.publish(pose_stamped_msg)

            self.frames[WaypointFrame].submit_waypoint_button_1.config(state=DISABLED)

     
            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")

    def submit_waypoint_1(self,x,y,z,yaw,path_orientation_style = 1,alg =2):
        """submit the single waypoint relative to current pose"""
        
        try:
        
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180


            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            self.desired_path.header.frame_id = 'NED'

            # formatting
            pose_msg = Pose()

            pose_msg = self.last_waypoint_rel_to_current_state_visualized
            transform = self.last_waypoint_rel_to_current_state_visualized_transform 
        
            # update the last waypoint transform
            self.last_waypoint_transform = transform

        
            # adding the waypoint to the waypoint array
            pose_stamped_msg.pose = pose_msg
            self.goal_waypoints.poses.append(pose_msg)

            # publishing the singular waypoint
            self.pub12.publish(pose_stamped_msg)

            # adding to path
            if not self.desired_path.poses:
                start_pose = PoseStamped()
                start_pose.header.frame_id = self.current_pose.header.frame_id
                start_pose.pose= self.current_pose.pose.pose
                self.desired_path.poses.append(start_pose)
    
            self.desired_path.poses.append(pose_stamped_msg)


            #  adding the orientation style along the path 
            path_orientation_style = self.entry_to_int(path_orientation_style)
            if path_orientation_style ==0:
                rospy.logwarn("No orientation style specified along the path, defaulting to TR")
                path_orientation_style = 1
            
            alg = self.entry_to_int(alg)
            if alg == 0:
                rospy.logwarn("No algorithm preference specified defaulting to alg 2")
                alg = 2

            # self.orientation_style.layout.dim.append(MultiArrayDimension())
            # self.orientation_style.data.append([path_orientation_style,alg])
            self.orientation_style.data.append(path_orientation_style)
            self.orientation_style.data.append(alg)
            self.pub15.publish(self.orientation_style)

            # rospy.loginfo(self.orientation_style)

            

            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
            self.pub5.publish(self.desired_path) # shows straight line path 

            self.frames[WaypointFrame].submit_waypoint_button_1.config(state=DISABLED)
            
            if not self.preset_patterns_window == None:
                self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
                self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
                self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
                self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
                
                # erase thevisualization and publish the empty arrays
                self.preset_pattern.poses.clear()
                self.preset_pattern_path.poses.clear()
                self.pub16.publish(self.preset_pattern)
                self.pub18.publish(self.preset_pattern_path)
         

        

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")
        
    def submit_waypoint_2(self,x,y,z,yaw,path_orientation_style = 1, alg = 2):
        """submit the single waypoint relative to last waypoint"""
        try:

            
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180


            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            # self.desired_path.header.frame_id = 'NED'

            # convert relative transform to global transform
            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)
            transform = concatenate_matrices(trans_matrix, rot_matrix) 
            transform = concatenate_matrices(self.last_waypoint_transform, transform)
            

            # convert to pose msg format
            translation = translation_from_matrix(transform)            
            quaternion = quaternion_from_matrix(transform)

            # updating pose_msg
            pose_msg = Pose()
            pose_msg.position.x = translation[0]
            pose_msg.position.y = translation[1]
            pose_msg.position.z = translation[2]
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]
            
            # update the last waypoint transform
            self.last_waypoint_transform = transform
        
            # adding the waypoint to the waypoint array
            pose_stamped_msg.pose = pose_msg
            self.goal_waypoints.poses.append(pose_msg)

            # publishing the singular waypoint
            self.pub12.publish(pose_stamped_msg)

            # adding to path, check if this is the first waypoint in the path and add start pose if needed
            if not self.desired_path.poses:
                start_pose = PoseStamped()
                start_pose.header.frame_id = self.current_pose.header.frame_id
                start_pose.pose= self.current_pose.pose.pose
                self.desired_path.poses.append(start_pose)


            

            # configs = self.dubin(self.desired_path.poses[-1], pose_stamped_msg)
            
    
            #  add to the desired straight line path
            self.desired_path.poses.append(pose_stamped_msg)


            #  adding the orientation style along the path 
            path_orientation_style = self.entry_to_int(path_orientation_style)
            if path_orientation_style ==0:
                rospy.logwarn("No orientation style specified aliong the path, defaulting to TR")
                path_orientation_style = 1

            alg = self.entry_to_int(alg)
            if alg == 0:
                rospy.logwarn("No algorithm preference specified defaulting to alg 2")
                alg = 2

            # self.orientation_style.layout.dim.append(MultiArrayDimension())
            # self.orientation_style.data.append([path_orientation_style,int(alg)])
            self.orientation_style.data.append(path_orientation_style)
            self.orientation_style.data.append(alg)

            # self.orientation_style.data.append(path_orientation_style)
            self.pub15.publish(self.orientation_style)

        
            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
            self.pub5.publish(self.desired_path) # shows straight line path 

            self.frames[WaypointFrame].submit_waypoint_button_1.config(state=DISABLED)

            # if not self.preset_patterns_window == None:
            #     self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
                
            #     # erase thevisualization and publish the empty arrays
            #     self.preset_pattern.poses.clear()
            #     self.preset_pattern_path.poses.clear()
            #     self.pub16.publish(self.preset_pattern)
            #     self.pub18.publish(self.preset_pattern_path)

            

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")
        
    def submit_waypoint_3(self,x,y,z,yaw,path_orientation_style = 1, alg=2):
        """submit the single waypoint relative to NED """
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180
          

            # update the transform for the waypoint
            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)


            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            self.desired_path.header.frame_id = 'NED'

            # formatting
            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            # previous way
            # transform = concatenate_matrices(rot_matrix, trans_matrix)

            # new_way
            transform = concatenate_matrices(trans_matrix, rot_matrix)

            
            # update the last waypoint transform
            self.last_waypoint_transform = transform

        
            # adding the waypoint to the waypoint array
            pose_stamped_msg.pose = pose_msg
            self.goal_waypoints.poses.append(pose_msg)

            # publishing the singular waypoint
            self.pub12.publish(pose_stamped_msg)

            # adding to path
            if not self.desired_path.poses:
                start_pose = PoseStamped()
                start_pose.header.frame_id = self.current_pose.header.frame_id
                start_pose.pose= self.current_pose.pose.pose
                self.desired_path.poses.append(start_pose)
    
            self.desired_path.poses.append(pose_stamped_msg)


            #  adding the orientation style along the path 
            path_orientation_style = self.entry_to_int(path_orientation_style)
            if path_orientation_style ==0:
                rospy.logwarn("No orientation style specified aliong the path, defaulting to TR")
                path_orientation_style = 4

            alg = self.entry_to_int(alg)

            if alg == 0:
                rospy.logwarn("No algorithm preference specified defaulting to alg 2")
                alg = 2

            # self.orientation_style.layout.dim.append(MultiArrayDimension())
            # self.orientation_style.data.append([path_orientation_style,int(alg)])

            self.orientation_style.data.append(path_orientation_style)
            self.orientation_style.data.append(alg)
            # self.orientation_style.data.append(path_orientation_style)
            self.pub15.publish(self.orientation_style)

        
            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
            self.pub5.publish(self.desired_path) # shows straight line path 


            # if not self.preset_patterns_window == None:
            #     self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
                
            #     # erase thevisualization and publish the empty arrays
            #     self.preset_pattern.poses.clear()
            #     self.preset_pattern_path.poses.clear()
            #     self.pub16.publish(self.preset_pattern)
            #     self.pub18.publish(self.preset_pattern_path)
            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")

    def submit_waypoint_4(self,lat,lon,z,yaw,path_orientation_style = 1,alg=2):
           
        # need to verify this, particularly the depth
        # since the distance is less than 100 m we can assume flat plane tangent
        # yaw is relative to starting yaw
      
        """Visualize the single waypoint relative to gps coord, sea level and starting yaw"""
        try:
            roll = 0
            pitch = 0
           
            lat = self.entry_to_float(lat)
            lon = self.entry_to_float(lon)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            # convert from alt lon to utm then get distance from ned origin UTM coord

            x_utm, y_utm, _, _= utm.from_latlon(lat, lon, force_northern=True)

            x = x_utm - self.init_x_utm
            y = y_utm - self.init_y_utm



            # x,y,z = self.wgs84_to_enu(self,lat, lon, alt)
            # z = alt

            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            self.desired_path.header.frame_id = 'NED'

            pose_msg = Pose()
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z


            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            # update the transform for the waypoint
            trans_matrix = translation_matrix([x, y, z])
            rot_matrix = euler_matrix(roll,pitch,yaw)       
            # new_way
            transform = concatenate_matrices(trans_matrix, rot_matrix)

            
            # update the last waypoint transform
            self.last_waypoint_transform = transform

        
            # adding the waypoint to the waypoint array
            pose_stamped_msg.pose = pose_msg
            self.goal_waypoints.poses.append(pose_msg)

            # publishing the singular waypoint
            self.pub12.publish(pose_stamped_msg)

            # adding to path
            if not self.desired_path.poses:
                start_pose = PoseStamped()
                start_pose.header.frame_id = self.current_pose.header.frame_id
                start_pose.pose= self.current_pose.pose.pose
                self.desired_path.poses.append(start_pose)
    
            self.desired_path.poses.append(pose_stamped_msg)


            #  adding the orientation style along the path 
            path_orientation_style = self.entry_to_int(path_orientation_style)
            if path_orientation_style == 0:
                rospy.logwarn("No orientation style specified aliong the path, defaulting to TR")
                path_orientation_style = 1
            
            alg = self.entry_to_int(alg)
            if alg == 0:
                rospy.logwarn("No algorithm preference specified defaulting to alg 2")
                alg = 2

            # self.orientation_style.layout.dim.append(MultiArrayDimension())
            # self.orientation_style.data.append([path_orientation_style,int(alg)])
            self.orientation_style.data.append(path_orientation_style)
            self.orientation_style.data.append(alg)
            # self.orientation_style.data.append(path_orientation_style)
            self.pub15.publish(self.orientation_style)

        
            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
            self.pub5.publish(self.desired_path) # shows straight line path 
            # if not self.preset_patterns_window == None:
            #     self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
            #     self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
                
            #     # erase thevisualization and publish the empty arrays
            #     self.preset_pattern.poses.clear()
            #     self.preset_pattern_path.poses.clear()
            #     self.pub16.publish(self.preset_pattern)
            #     self.pub18.publish(self.preset_pattern_path)
                
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")

    def erase_waypoints(self):
        """ Erase the waypoints array """
        self.goal_waypoints.poses.clear()
        self.desired_path.poses.clear()
        self.orientation_style.data.clear()
            
        self.hold_pose = False # it changes to true inside the next function call
        self.hold_pose_button_function()


        self.pub13.publish(True)
        
        self.pub15.publish(self.orientation_style)
        self.pub3.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
        self.pub15.publish(self.orientation_style)
    
        rospy.loginfo("Erased all waypoints, holding current position")
    
    def hold_pose_button_function(self):
        
        if not self.hold_pose:
            self.hold_pose = True
            
        elif self.hold_pose:
            self.hold_pose = False

        self.pub7.publish(self.hold_pose)
       
    def submit_pos_threshold(self, threshold):
        try:
            indicator = 1
            rospy.loginfo("Setting position threshold")
            threshold = self.entry_to_float(threshold)

            self.path_planner_msgs.data = [
                    indicator,
                    threshold
                    ]
            self.pub17.publish(self.path_planner_msgs)

        except: 
            rospy.logerr("Encountered an error in submit_pos_threshold")
    
    def submit_orientation_threshold(self, threshold):
        try:
            indicator = 2
            rospy.loginfo("Setting orientation threshold")
            threshold = self.entry_to_float(threshold)

            self.path_planner_msgs.data = [
                    indicator,
                    threshold
                    ]
            
            self.pub17.publish(self.path_planner_msgs)
            
        except: 
            rospy.logerr("Encountered an error in submit_orientation_threshold")
    
    def submit_lookahead_distance(self, distance):
        try:
            indicator = 3
            rospy.loginfo("Setting lookahead distance")
            distance= self.entry_to_float(distance)

            self.path_planner_msgs.data = [
                    indicator,
                    distance
                    ]
            
            self.pub17.publish(self.path_planner_msgs)
            
        except: 
            rospy.logerr("Encountered an error in submit_lookahead_distance")
    
    def submit_lookahead_distance2(self, distance):
        try:
            indicator = 4
            rospy.loginfo("Setting lookahead distance past waypoint")
            distance= self.entry_to_float(distance)

            self.path_planner_msgs.data = [
                    indicator,
                    distance
                    ]
            
            self.pub17.publish(self.path_planner_msgs)
            
        except: 
            rospy.logerr("Encountered an error in submit_lookahead_distance2")
        
    def submit_algorithm(self, method):
        try:
            indicator = 5
            rospy.loginfo("Setting path_following algorithm")
            method = self.entry_to_int(method)

            self.path_planner_msgs.data = [
                    indicator,
                    method
                    ]
            
            self.pub17.publish(self.path_planner_msgs)
            
        except: 
            rospy.logerr("Encountered an error in submit_lookahead_distance2")
        
    def submit_controller_gains(self, indicator,kpx, kdx, kix,
             kpy, kdy, kiy,
             kpz, kdz, kiz,
             kpyaw, kdyaw, kiyaw):
    
        try:
            if indicator ==1:
                rospy.loginfo("Sending controller gains for position controller")
            elif indicator ==2:
                rospy.loginfo("Sending controller gains for velcoity controller")
                            
            self.motion_controller_msgs.data = [
                    indicator,
                    self.entry_to_float(kpx),
                    self.entry_to_float(kdx),
                    self.entry_to_float(kix),
                    self.entry_to_float(kpy),
                    self.entry_to_float(kdy),
                    self.entry_to_float(kiy),
                    self.entry_to_float(kpz),
                    self.entry_to_float(kdz),
                    self.entry_to_float(kiz),
                    self.entry_to_float(kpyaw),
                    self.entry_to_float(kdyaw),
                    self.entry_to_float(kiyaw),
                ]
            self.pub9.publish(self.motion_controller_msgs)

        except ValueError as ve:
            rospy.logerr(f"Invalid input for gains: {ve}")
    
    def submit_max_pwm(self, pwm):
        try:
            indicator = 4
            rospy.loginfo("Setting max pwm signal")
            max_pwm  = float(pwm)
            self.motion_controller_msgs.data = [
                    indicator,
                    max_pwm
                    ]
            self.pub9.publish(self.motion_controller_msgs)
        except: 
            rospy.logerr("Encountered an error in submit_max_pwm")
        
    def submit_max_linear_velocity(self,velocity):
        try:
            indicator = 3
            rospy.loginfo("Seting max linear velocity")
            # max_linear_velocity  = float(velocity)
            self.motion_controller_msgs.data = [
                    indicator,
                    float(velocity)
                    ]
            self.pub9.publish(self.motion_controller_msgs)
        except:
            rospy.logerr("Encountered an error in submit_max_linear_velocity")
            
    def submit_max_angular_velocity(self,velocity):
        try:
            indicator = 9
            rospy.loginfo("Seting max angular velocity")
            # self.max_angular_velocity  = float(velocity)
            self.motion_controller_msgs.data = [
                    indicator,
                    float(velocity)
                    ]
            self.pub9.publish(self.motion_controller_msgs)
        except:
            rospy.logerr("Encountered an error in submit_max_angular_velocity")
     
    def entry_to_float(self,var):
        try:
            var = float(var)
        except:
            var = 0.0
        return var 
    
    def entry_to_int(self,var):
        try:
            var = int(var)
        except:
            var = 0
        return var 

    def visualize_path(self, path_indicator, orientation_indicator):
        # rospy.loginfo(int(path_indicator))
        self.path_generation_data.data = [int(path_indicator), int(orientation_indicator)]
        self.pub14.publish(self.path_generation_data)
        rospy.loginfo("Requesting path visualization")
            
    def generate_square(self, relative_indicator, size, rotate_direction, path_orientation_style,alg):
        size = self.entry_to_float(size)
        if size == 0: 
            rospy.logwarn("Cannot create square of size 0, defaulting to 1 m ")
            size = 1 
          
        if relative_indicator == "1":
            
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))

            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)
                        
               
        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0, path_orientation_style,alg) 
           
        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))

            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)
                 
        
        # if relative_indicator == "1":
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        # elif relative_indicator =="2":
        #     self.submit_waypoint_2(0,0,0,0, path_orientation_style)
        # elif relative_indicator =="":
        #     rospy.logwarn("start point not selected, defaulting to current pose")
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)

        if rotate_direction == "1":
            rotate = 90
        elif rotate_direction == "2":
            rotate = -90
        elif relative_indicator =="":
            rospy.logwarn("rotation direction not selected, defaulting to cw")
            rotate = 90 
        else: 
            rotate = 90
    

        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style,alg)
        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style,alg)
        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style,alg)
        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style,alg)

        # buttons are disabled until the next pattern is visualized
        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
        
        # erase thevisualization and publish the empty arrays
        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)
        
    def generate_orbit(self, relative_indicator, radius, rotation_angle, rotation_direction,path_orientation_style,alg):
        radius = self.entry_to_float(radius)
        rotation_angle= self.entry_to_float(rotation_angle)*np.pi/180

        if radius == 0: 
            rospy.logwarn("Cannot create circle of radius 0, defaulting to 1 m ")
            radius = 1 


        if relative_indicator == "1":
            
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))

            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)

        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0, path_orientation_style,alg)
                   
        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))
            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)



        # if relative_indicator == "1":
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0,path_orientation_style)

        # elif relative_indicator =="2":
        #     self.submit_waypoint_2(0,0,0,0,path_orientation_style)

        # elif relative_indicator =="":
        #     rospy.logwarn("start point not selected defaulting to current pose")
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        
        if rotation_direction =="1":
            direction_sign = 1

        elif rotation_direction == "2":
            direction_sign = -1
            
        elif rotation_direction == "":
               rospy.logwarn("Rotation direction was not specified, defaulting to cw")
               direction_sign = 1



        # determine how many points you want on the circle
        interval = 15*np.pi/180
        
        if rotation_angle == 0:
            rospy.logwarn("Cannot create circle of angle 0, defaulting to 360 degrees ")
            rotation_angle = 360 *np.pi/180
            
        elif rotation_angle < interval:
            interval = rotation_angle

        
        num_points = int(rotation_angle/interval)+1


        # find center of circle 
        trans_matrix = translation_matrix([radius,0,0])
        transform = concatenate_matrices(self.last_waypoint_transform, trans_matrix)

        # get starting angle
        x,y,z = translation_from_matrix(transform) 
        roll,pitch,yaw = euler_from_matrix(transform)

        # normalize starting angle
        start_angle = yaw + np.pi
        if start_angle > np.pi:
            start_angle -= 2 * np.pi
        elif start_angle < -np.pi:
            start_angle += 2 * np.pi

    

        #avoid re-adding the first waypoint
        i = 1 
        # while i < num_points:
        for i in range(1, num_points):    

            # get angle and normailize it
            angle = start_angle+(i * interval * direction_sign)
            if angle > np.pi:
                angle -= 2 * np.pi
            elif angle < -np.pi:
                angle += 2 * np.pi
        
            new_x = x + (radius*np.cos(angle))
            new_y = y + (radius*np.sin(angle))


            # get yaw of new point
            yaw = angle + np.pi
            if yaw > np.pi:
                yaw -= 2 * np.pi
            elif yaw < -np.pi:
                yaw += 2 * np.pi

            # convert to degrees
            yaw = yaw *180/np.pi

            self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style,alg)
            i+=1
        
        # add the last waypoint at exact desired location
        angle = start_angle+(rotation_angle * direction_sign)
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
      
        
        new_x = x + (radius*np.cos(angle))
        new_y = y + (radius*np.sin(angle))
        
        # get yaw of last point in degrees
        yaw = (angle+np.pi)*180/np.pi

        self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style,alg)  

        # buttons are disabled until the next pattern is visualized
        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
        
        # erase thevisualization and publish the empty arrays
        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)   
    
    def generate_arc(self, relative_indicator, radius, rotation_angle, rotation_direction,path_orientation_style,alg):
        radius = self.entry_to_float(radius)
        rotation_angle= self.entry_to_float(rotation_angle)*np.pi/180

        if radius == 0: 
            rospy.logwarn("Cannot create circle of radius 0, defaulting to 1 m ")
            radius = 1 

        # if relative_indicator == "1":
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)

        # elif relative_indicator =="2":
        #     self.submit_waypoint_2(0,0,0,0, path_orientation_style)

        # elif relative_indicator =="":
        #     rospy.logwarn("start point not selected defaulting to current pose")
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)


        if relative_indicator == "1":
            
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))
            # rospy.loginf(f"Start_yaw = ")
            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)

        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0, path_orientation_style,alg)
                   
        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))
            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)

        

        #  get last waypoint transform
        roll, pitch , yaw = euler_from_matrix(self.last_waypoint_transform)

        if rotation_direction =="1":
            direction_sign = 1
            start_angle = yaw -np.pi/2

        elif rotation_direction == "2":
            direction_sign = -1
            start_angle = yaw + np.pi/2
            
        elif rotation_direction == "":
            rospy.logwarn("Rotation direction was not specified, defaulting to cw")
            direction_sign = 1
            start_angle = yaw - np.pi/2
      
        # normalize starting angle
        if start_angle > np.pi:
            start_angle -= 2 * np.pi
        elif start_angle < -np.pi:
            start_angle += 2 * np.pi



        # determine how many points you want on the circle
        interval = 15 * np.pi/180
        
        if rotation_angle == 0:
            rospy.logwarn("Cannot create circle of angle 0, defaulting to 90 degrees ")
            rotation_angle = 90 *np.pi/180
            
        elif rotation_angle < interval:
            interval = rotation_angle

        
        num_points = int(rotation_angle/interval)+1

        # find center of circle 
        trans_matrix = translation_matrix([0,radius*direction_sign,0])
        transform = concatenate_matrices(self.last_waypoint_transform, trans_matrix)
        x, y, z = translation_from_matrix(transform) 
        
        # debugging center point
        # self.submit_waypoint_3(x, y, z, start_angle*180/np.pi)   
    

        #avoid re-adding the first waypoint
        for i in range (1, num_points):

            # get angle and normailize it
            angle = start_angle+(i * interval * direction_sign)
            if angle > np.pi:
                angle -= 2 * np.pi
            elif angle < -np.pi:
                angle += 2 * np.pi
        
            # get new coordinates
            new_x = x + (radius*np.cos(angle))
            new_y = y + (radius*np.sin(angle))


            # get yaw of new point
            if direction_sign == 1:
                yaw = angle + np.pi/2
            elif direction_sign == -1:
                yaw = angle - np.pi/2


            if yaw > np.pi:
                yaw -= 2 * np.pi
            elif yaw < -np.pi:
                yaw += 2 * np.pi

            # convert to degrees
            yaw = yaw *180/np.pi
    
            self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style,alg)
            
        
        # add the last waypoint at exact desired location
        angle = start_angle+(rotation_angle * direction_sign)
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
      
        
        new_x = x + (radius*np.cos(angle))
        new_y = y + (radius*np.sin(angle))
        
        # get yaw of last point in degrees
        yaw = (angle+np.pi/2)*180/np.pi
        self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style, alg)   

        # buttons are disabled until the next pattern is visualized
        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
        
        # erase thevisualization and publish the empty arrays
        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)
     
    def generate_lawnmower(self, relative_indicator, leg_length, leg_spacing, n_legs, rotate_direction, path_orientation_style,alg):
      
        leg_length = self.entry_to_float(leg_length)
        leg_spacing = self.entry_to_float(leg_spacing)
        n_legs = self.entry_to_int(n_legs)

        if leg_length == 0: 
            rospy.logwarn("Cannot create leg length of size 0, defaulting to 1 m ")
            leg_length = 1 

        if leg_spacing == 0:
            rospy.logwarn("Cannot create leg spacing of size 1, defaulting to 1 m ")
            leg_spacing  = 1

        if n_legs == 0:
            rospy.logwarn("Cannot create 0 legs, defaulting to 2 legs ")
            n_legs = 2

        if rotate_direction == "1":
            rotate = 90
        elif rotate_direction == "2":
            rotate = -90
        
        elif rotate_direction =="":
            rospy.logwarn("rotation direction not selected, defaulting to cw")
            rotate = 90 
        
  

        if relative_indicator == "1":
            
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))

            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)
                        
            # self.visualize_waypoint_1(0,0,0,0)
            # self.submit_waypoint_1(0,0,0,0, path_orientation_style)
            self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style)
            
        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0, path_orientation_style,alg)
            self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style,alg)

        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            _,__,start_yaw = euler_from_quaternion((self.preset_pattern.poses[0].orientation.x,self.preset_pattern.poses[0].orientation.y,self.preset_pattern.poses[0].orientation.z, self.preset_pattern.poses[0].orientation.w))

            self.submit_waypoint_3(self.preset_pattern.poses[0].position.x,self.preset_pattern.poses[0].position.y,self.preset_pattern.poses[0].position.z, start_yaw*180/np.pi, path_orientation_style,alg)
                 
            # self.visualize_waypoint_1(0,0,0,0)
            # self.submit_waypoint_1(0,0,0,0, path_orientation_style)
            self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style,alg)
            

        # self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style)

    

        # # new method
        # if relative_indicator == "1":
        #     # self.visualize_waypoint_1(leg_length,0,0,rotate)
        #     # self.submit_waypoint_1(leg_length,0,0,rotate, path_orientation_style)

        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        #     self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style)
            
        # elif relative_indicator =="2":
        #     self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style)
        # elif relative_indicator =="":
        #     rospy.logwarn("start point not selected, defaulting to current pose")
        #     self.visualize_waypoint_1(leg_length,0,0,rotate)
        #     self.submit_waypoint_1(leg_length,0,0,rotate, path_orientation_style)

       

        for i in range(1,n_legs):
            self.submit_waypoint_2(leg_spacing,0,0,rotate, path_orientation_style,alg)
            if i == n_legs-1:
                rotate = 0
            else:
                rotate *=-1
            self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style,alg)

    
        # buttons are disabled until the next pattern is visualized
        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
        
        # erase thevisualization and publish the empty arrays
        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)
        
    def visualize_lawnmower(self, relative_indicator, leg_length, leg_spacing, n_legs, rotate_direction, path_orientation_style):
        
        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
       
        leg_length = self.entry_to_float(leg_length)
        leg_spacing = self.entry_to_float(leg_spacing)
        n_legs = self.entry_to_int(n_legs)

        if leg_length == 0: 
            rospy.logwarn("Cannot create leg length of size 0, defaulting to 1 m ")
            leg_length = 1 

        if leg_spacing == 0:
            rospy.logwarn("Cannot create leg spacing of size 1, defaulting to 1 m ")
            leg_spacing  = 1 

        if n_legs == 0:
            rospy.logwarn("Cannot create 0 legs, defaulting to 2 legs ")
            n_legs = 2

        if rotate_direction == "1":
            rotate = 90
        elif rotate_direction == "2":
            rotate = -90
        elif rotate_direction =="":
            rospy.logwarn("rotation direction not selected, defaulting to cw")
            rotate = 90 
        
  

        if relative_indicator == "1":
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)

            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(leg_length,0,0,rotate, last_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
            
        elif relative_indicator =="2":
           
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(leg_length,0,0,rotate, self.last_waypoint_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
          

        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)

            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(leg_length,0,0,rotate, last_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)

        
        for i in range(1,n_legs):
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(leg_spacing,0,0,rotate, last_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
        
            if i == n_legs-1:
                rotate = 0
            else:
                rotate *=-1
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(leg_length, 0,0,rotate, last_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)

        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)

        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=NORMAL)

        # buttons are disabled until the next pattern is visualized
        # self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
        
    def visualize_square(self, relative_indicator, size, rotate_direction, path_orientation_style):

        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
        
        size = self.entry_to_float(size)
        if size == 0: 
            rospy.logwarn("Cannot create square of size 0, defaulting to 1 m ")
            size = 1 

        # if relative_indicator == "1":
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        # elif relative_indicator =="2":
        #     self.submit_waypoint_2(0,0,0,0, path_orientation_style)
        # elif relative_indicator =="":
        #     rospy.logwarn("start point not selected, defaulting to current pose")
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)

        
        if relative_indicator == "1":
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
            
        elif relative_indicator =="2":
           
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(0,0,0,0,self.last_waypoint_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
          

        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)


        if rotate_direction == "1":
            rotate = 90
        elif rotate_direction == "2":
            rotate = -90
        elif relative_indicator =="":
            rospy.logwarn("rotation direction not selected, defaulting to cw")
            rotate = 90 
        else: 
            rotate = 90
    
        waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(size,0,0,rotate, last_transform)
        self.preset_pattern.poses.append(waypoint.pose)
        self.preset_pattern_path.poses.append(waypoint)
        
        waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(size,0,0,rotate, last_transform)
        self.preset_pattern.poses.append(waypoint.pose)
        self.preset_pattern_path.poses.append(waypoint)
        
        waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(size,0,0,rotate, last_transform)
        self.preset_pattern.poses.append(waypoint.pose)
        self.preset_pattern_path.poses.append(waypoint)
        
        waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(size,0,0,rotate, last_transform)
        self.preset_pattern.poses.append(waypoint.pose)
        self.preset_pattern_path.poses.append(waypoint)


        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)

        self.preset_patterns_window.submit_generate_square_path_button.configure(state=NORMAL)


        # buttons are disabled until the next pattern is visualized
        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        # self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
                
    def visualize_arc(self, relative_indicator, radius, rotation_angle, rotation_direction,path_orientation_style):
        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
        
        radius = self.entry_to_float(radius)
        rotation_angle= self.entry_to_float(rotation_angle)*np.pi/180

        if radius == 0: 
            rospy.logwarn("Cannot create circle of radius 0, defaulting to 1 m ")
            radius = 1 

        if relative_indicator == "1":
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
            
            # self.visualize_waypoint_1(0,0,0,0)
            # self.submit_waypoint_1(0,0,0,0, path_orientation_style)

        elif relative_indicator =="2":
            # self.submit_waypoint_2(0,0,0,0, path_orientation_style)
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(0,0,0,0,self.last_waypoint_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)

        elif relative_indicator =="":
            rospy.logwarn("start point not selected defaulting to current pose")
            # self.visualize_waypoint_1(0,0,0,0)
            # self.submit_waypoint_1(0,0,0,0, path_orientation_style)
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
        

        #  get last waypoint transform
        roll, pitch , yaw = euler_from_matrix(last_transform)

        if rotation_direction =="1":
            direction_sign = 1
            start_angle = yaw -np.pi/2

        elif rotation_direction == "2":
            direction_sign = -1
            start_angle = yaw + np.pi/2
            
        elif rotation_direction == "":
            rospy.logwarn("Rotation direction was not specified, defaulting to cw")
            direction_sign = 1
            start_angle = yaw - np.pi/2
      
        # normalize starting angle
        if start_angle > np.pi:
            start_angle -= 2 * np.pi
        elif start_angle < -np.pi:
            start_angle += 2 * np.pi



        # determine how many points you want on the circle
        interval = 15 * np.pi/180
        
        if rotation_angle == 0:
            rospy.logwarn("Cannot create circle of angle 0, defaulting to 90 degrees ")
            rotation_angle = 90 *np.pi/180
            
        elif rotation_angle < interval:
            interval = rotation_angle

        
        num_points = int(rotation_angle/interval)+1

        # find center of circle 
        trans_matrix = translation_matrix([0,radius*direction_sign,0])
        transform = concatenate_matrices(last_transform, trans_matrix)
        x, y, z = translation_from_matrix(transform) 
        
        # debugging center point
        # self.submit_waypoint_3(x, y, z, start_angle*180/np.pi)   
    

        #avoid re-adding the first waypoint
        # i = 1 
        # while i < num_points:  
        for i in range(1,num_points): 

            # get angle and normailize it
            angle = start_angle+(i * interval * direction_sign)
            if angle > np.pi:
                angle -= 2 * np.pi
            elif angle < -np.pi:
                angle += 2 * np.pi
        
            # get new coordinates
            new_x = x + (radius*np.cos(angle))
            new_y = y + (radius*np.sin(angle))


            # get yaw of new point
            if direction_sign == 1:
                yaw = angle + np.pi/2
            elif direction_sign == -1:
                yaw = angle - np.pi/2


            if yaw > np.pi:
                yaw -= 2 * np.pi
            elif yaw < -np.pi:
                yaw += 2 * np.pi

            # convert to degrees
            yaw = yaw *180/np.pi
           
            waypoint = self.preset_pattern_visualize_waypoint_3(new_x, new_y, z, yaw)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
          
    
            # self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style)
            # i+=1
        
        # add the last waypoint at exact desired location
        angle = start_angle+(rotation_angle * direction_sign)
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
      
        
        new_x = x + (radius*np.cos(angle))
        new_y = y + (radius*np.sin(angle))
        
        # get yaw of last point in degrees
        yaw = (angle+np.pi/2)*180/np.pi

        waypoint = self.preset_pattern_visualize_waypoint_3(new_x, new_y, z, yaw)
        self.preset_pattern.poses.append(waypoint.pose)
        self.preset_pattern_path.poses.append(waypoint)

        # self.submit_waypoint_3(new_x, new_y, z, yaw)     

        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)

 

        # buttons are disabled until the next pattern is visualized
        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=NORMAL)
     
    def visualize_orbit(self, relative_indicator, radius, rotation_angle, rotation_direction,path_orientation_style):
        self.preset_pattern.poses.clear()
        self.preset_pattern_path.poses.clear()
        
        radius = self.entry_to_float(radius)
        rotation_angle= self.entry_to_float(rotation_angle)*np.pi/180

        if radius == 0: 
            rospy.logwarn("Cannot create circle of radius 0, defaulting to 1 m ")
            radius = 1 
            
        if relative_indicator == "1":
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
    

        elif relative_indicator =="2":
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_2(0,0,0,0,self.last_waypoint_transform)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)

        elif relative_indicator =="":
            rospy.logwarn("start point not selected defaulting to current pose")
            waypoint, last_transform = self.preset_pattern_visualize_waypoint_1(0,0,0,0)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
        



        # if relative_indicator == "1":
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0,path_orientation_style)

        # elif relative_indicator =="2":
        #     self.submit_waypoint_2(0,0,0,0,path_orientation_style)

        # elif relative_indicator =="":
        #     rospy.logwarn("start point not selected defaulting to current pose")
        #     self.visualize_waypoint_1(0,0,0,0)
        #     self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        
        if rotation_direction =="1":
            direction_sign = 1

        elif rotation_direction == "2":
            direction_sign = -1
            
        elif rotation_direction == "":
               rospy.logwarn("Rotation direction was not specified, defaulting to cw")
               direction_sign = 1



        # determine how many points you want on the circle
        interval = 15*np.pi/180
        
        if rotation_angle == 0:
            rospy.logwarn("Cannot create circle of angle 0, defaulting to 360 degrees ")
            rotation_angle = 360 *np.pi/180
            
        elif rotation_angle < interval:
            interval = rotation_angle

        
        num_points = int(rotation_angle/interval)+1


        # find center of circle 
        trans_matrix = translation_matrix([radius,0,0])
        transform = concatenate_matrices(last_transform, trans_matrix)

        # get starting angle
        x,y,z = translation_from_matrix(transform) 
        roll,pitch,yaw = euler_from_matrix(transform)

        # normalize starting angle
        start_angle = yaw + np.pi
        if start_angle > np.pi:
            start_angle -= 2 * np.pi
        elif start_angle < -np.pi:
            start_angle += 2 * np.pi

    

        #avoid re-adding the first waypoint
        for i in range(1, num_points):
            # get angle and normailize it
            angle = start_angle+(i * interval * direction_sign)
            if angle > np.pi:
                angle -= 2 * np.pi
            elif angle < -np.pi:
                angle += 2 * np.pi
        
            new_x = x + (radius*np.cos(angle))
            new_y = y + (radius*np.sin(angle))


            # get yaw of new point
            yaw = angle + np.pi
            if yaw > np.pi:
                yaw -= 2 * np.pi
            elif yaw < -np.pi:
                yaw += 2 * np.pi

            # convert to degrees
            yaw = yaw *180/np.pi

            waypoint = self.preset_pattern_visualize_waypoint_3(new_x, new_y, z, yaw)
            self.preset_pattern.poses.append(waypoint.pose)
            self.preset_pattern_path.poses.append(waypoint)
            
            # self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style)
            # i+=1
        
        # add the last waypoint at exact desired location
        angle = start_angle+(rotation_angle * direction_sign)
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
      
        
        new_x = x + (radius*np.cos(angle))
        new_y = y + (radius*np.sin(angle))
        
        # get yaw of last point in degrees
        yaw = (angle+np.pi)*180/np.pi

        # self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style)    
        waypoint = self.preset_pattern_visualize_waypoint_3(new_x, new_y, z, yaw)
        self.preset_pattern.poses.append(waypoint.pose)
        self.preset_pattern_path.poses.append(waypoint) 


        self.pub16.publish(self.preset_pattern)
        self.pub18.publish(self.preset_pattern_path)

 

        # buttons are disabled until the next pattern is visualized
        self.preset_patterns_window.submit_generate_lawnmower_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_square_path_button.configure(state=DISABLED)
        self.preset_patterns_window.submit_generate_orbit_path_button.configure(state=NORMAL)
        self.preset_patterns_window.submit_generate_arc_path_button.configure(state=DISABLED)
    
    # used specifically to be able to visualize the preset patterns
    def preset_pattern_visualize_waypoint_2(self,x,y,z,yaw,last_wp_transform):
      
        """Visualize the single waypoint relative to last waypoint added"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'


            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)

            transform = concatenate_matrices(trans_matrix, rot_matrix) #makes translation in current frame then rotates the orientationof the body
            transform = concatenate_matrices(last_wp_transform, transform)


            # convert to pose msg format
            translation = translation_from_matrix(transform) 
            quaternion = quaternion_from_matrix(transform)
           
            # updating pose_msg
            pose_msg.position.x = translation[0] 
            pose_msg.position.y = translation[1]
            pose_msg.position.z = translation[2]
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]


            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
            # self.pub1.publish(pose_stamped_msg)

            return pose_stamped_msg, transform


            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")
    
    def preset_pattern_visualize_waypoint_1(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to current pose"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            # update the transform for the waypoint
            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)
            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            
            # convert relative transform relative to body frame to Ned frame
            trans_matrix = translation_matrix([x,y,z])
            rot_matrix = euler_matrix(roll,pitch,yaw)    
            transform = concatenate_matrices(trans_matrix, rot_matrix)            
            transform = concatenate_matrices(self.get_current_pose_transform(), transform)

            # convert to pose msg format
            translation = translation_from_matrix(transform)            
            quaternion = quaternion_from_matrix(transform)

            # updating pose_msg
            pose_msg.position.x = translation[0]
            pose_msg.position.y = translation[1]
            pose_msg.position.z = translation[2]
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]

            # self.last_waypoint_rel_to_current_state_visualized  = pose_msg
            # self.last_waypoint_rel_to_current_state_visualized_transform = transform
        
            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
           

            # self.pub1.publish(pose_stamped_msg)

            # self.frames[WaypointFrame].submit_waypoint_button_1.config(state=NORMAL)
            return pose_stamped_msg, transform


        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")


            # messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")

    def preset_pattern_visualize_waypoint_3(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to NED"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180

            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            
        
            pose_msg.position.x = x
            pose_msg.position.y = y
            pose_msg.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_msg.orientation.x = quaternion[0]
            pose_msg.orientation.y = quaternion[1]
            pose_msg.orientation.z = quaternion[2]
            pose_msg.orientation.w = quaternion[3]
            

            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
            # self.pub1.publish(pose_stamped_msg)

            

            # self.frames[WaypointFrame].submit_waypoint_button_1.config(state=DISABLED)
            return pose_stamped_msg
        
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")
 
    def wgs84_to_ecef(self,lat, lon, alt):
        # constant from fossen pg 36 table 2.2
        # WGS84 ellipsoid constants
        a = 6378137    # semi-major axis (meters)
        b = 6356752  # semi-minor axis (meters)
        e2 = (a**2 - b**2) / a**2  # first eccentricity squared 
        # e = 0.0818 # first eccentricity of ellipsoid
        # print(f"e2: ={e2}")
    
        lat = lat*np.pi/180
        lon = lon*np.pi/180


        # fossen page36 eq 2.88
        N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)

        X = (N + alt) * np.cos(lat) * np.cos(lon)
        Y = (N + alt) * np.cos(lat) * np.sin(lon)
        # Z = (N * (1 - e2) + alt) * np.sin(lat)
        Z = ((b**2)*N/(a**2)+ alt) * np.sin(lat)

        return X, Y, Z

    def ecef_to_enu(self,x, y, z):
        # Convert reference position to ECEF
        x_ref, y_ref, z_ref = self.wgs84_to_ecef(self.lat_ref, self.lon_ref, self.alt_ref)

        # Translation to the reference point
        dx = x - x_ref
        dy = y - y_ref
        dz = z - z_ref

        # Reference latitude and longitude in radians
        lat_ref = lat_ref*np.pi/180
        lon_ref = lon_ref*np.pi/180

        # Create the rotation matrix
        R = np.array([
            [-np.sin(lon_ref),  np.cos(lon_ref), 0],
            [-np.sin(lat_ref) * np.cos(lon_ref), -np.sin(lat_ref) * np.sin(lon_ref), np.cos(lat_ref)],
            [ np.cos(lat_ref) * np.cos(lon_ref),  np.cos(lat_ref) * np.sin(lon_ref), np.sin(lat_ref)]
        ])

        # Apply the rotation
        enu = R.dot(np.array([dx, dy, dz]))

        return enu

    def wgs84_to_enu(self,lat, lon, alt):
        x, y, z = self.wgs84_to_ecef(lat, lon, alt)
        enu = self.ecef_to_enu(x, y, z)
        return enu

    def dubin(self, start:PoseStamped, end:PoseStamped):

        _,_,start_yaw  = euler_from_quaternion([start.pose.orientation.x, start.pose.orientation.y,start.pose.orientation.z,start.pose.orientation.w])
        _,_,end_yaw  = euler_from_quaternion([end.pose.orientation.x, end.pose.orientation.y,end.pose.orientation.z,end.pose.orientation.w])
        q0 = (start.pose.position.x, start.pose.position.y,start_yaw)
        q1 = (end.pose.position.x, end.pose.position.y,end_yaw)
        turning_radius = 1
        step_size = 0.5

        path = dubins.shortest_path(q0, q1, turning_radius)

     
        configurations, _ = path.sample_many(step_size)

        
        for wp in configurations:
                  
            new_wp = PoseStamped()
            new_wp.pose.position.x = wp[0]
            new_wp.pose.position.y = wp[1]
            new_wp.pose.position.z = start.pose.position.z

            q = quaternion_from_euler(0,0,wp[2])

            new_wp.pose.orientation.x = q[0]
            new_wp.pose.orientation.y = q[1]
            new_wp.pose.orientation.z = q[2]
            new_wp.pose.orientation.w = q[3]

            self.goal_waypoints.poses.append(new_wp.pose)
            self.desired_path.poses.append(new_wp)
        
        self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
        self.pub5.publish(self.desired_path) # shows straight line path 

        # print(configurations)
        return configurations


    def submit_current_pose(self,x,y,z,yaw,path_orientation_style = 1, alg=2):
        """submit the single waypoint relative to NED """
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.entry_to_float(x)
            y = self.entry_to_float(y)
            z = self.entry_to_float(z)
            yaw = self.entry_to_float(yaw)*np.pi / 180
          

          

            # formatting for rviz visualization
            pose_stamped_msg = PoseWithCovarianceStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            pose_stamped_msg.pose.pose.position.x = x
            pose_stamped_msg.pose.pose.position.y = y
            pose_stamped_msg.pose.pose.position.z = z

            quaternion = quaternion_from_euler(roll, pitch, yaw)
            pose_stamped_msg.pose.pose.orientation.x = quaternion[0]
            pose_stamped_msg.pose.pose.orientation.y = quaternion[1] 
            pose_stamped_msg.pose.pose.orientation.z = quaternion[2]
            pose_stamped_msg.pose.pose.orientation.w = quaternion[3]

            # publishing the singular waypoint
            self.pub19.publish(pose_stamped_msg)

            self.br.sendTransform(
                (x,y,z),
                quaternion,
                rospy.Time.now(),
                'base_link',   # child frame
                'NED'        # parent frame, typically 'world' or 'map'
            )

    
            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")


# frames in main window
class InfoPage(LabelFrame):
    def __init__(self, parent, controller):
        LabelFrame.__init__(self, parent,text="Information")
        label = Label(self, text="BlueROV2 Heavy Configuration Interface\nField Robotics Group\nUniversity of Michigan\nAndrew Albano")
        label.grid(row=0, column=0, padx=10, pady=10)

class ControlOptionsPage(LabelFrame):
    def __init__(self, parent, controller):
        LabelFrame.__init__(self, parent,text="Control Modes")


        row_index = 0
        self.open_joystick_mode_button = Button(self, text="Activate Joystick Mode", command=lambda: controller.show_frame(JoystickFrame))
        self.open_joystick_mode_button.grid(row=row_index, column= 0,columnspan=1,padx=20, pady=10,sticky="ew")
        row_index+=1
        
        self.open_waypoint_mode_button = Button(self, text="Activate Waypoint Mode", command=lambda: controller.show_frame(WaypointFrame))
        self.open_waypoint_mode_button.grid(row=row_index, column= 0,columnspan=1,padx=20, pady=10,sticky="ew")
        row_index+=1
        
        self.open_velocity_mode_button = Button(self, text="Activate Velocity Mode", command=lambda: controller.show_frame(VelocityFrame))
        self.open_velocity_mode_button.grid(row=row_index, column= 0, columnspan=1, padx=20, pady=10,sticky="ew")
        row_index+=1

        self.open_pwm_mode_button = Button(self, text="Activate PWM Mode", command=lambda: controller.show_frame(PWMFrame))
        self.open_pwm_mode_button.grid(row=row_index, column = 0, columnspan=1, padx=20, pady=10,sticky="ew")
        row_index+=1


    def open_controller_params_button_function(self,controller):
        if not controller.open_controller_params_frame:
            controller.open_controller_params_frame = True
            text = "Close Controller Parameters"
        elif controller.open_controller_params_frame:    
            controller.open_controller_params_frame  = False
            text = "Edit Controller Parameters"
            

        self.open_controller_params_button.configure(text=f"{text}")

        controller.open_close_controller_params_window()

class JoystickFrame(LabelFrame):
    def __init__(self, parent, controller):
        LabelFrame.__init__(self, parent,text="Joystick Mode")

        row_index =0
        self.close_joystick_mode_button = Button(self, text="Disable Joystick Mode", command=lambda: controller.show_frame(ControlOptionsPage))
        self.close_joystick_mode_button.grid(row=row_index, column= 0, columnspan=1, padx=20, pady=10,sticky="ew")
        row_index+=1

class VelocityFrame(LabelFrame):
    def __init__(self, parent, controller):
        LabelFrame.__init__(self, parent,text="Velocity Mode")

        row_index =0
        self.close_velocity_mode_button = Button(self, text="Disable Velocity Mode", command=lambda: controller.show_frame(ControlOptionsPage))
        self.close_velocity_mode_button.grid(row=row_index, column= 0, columnspan=1, padx=20, pady=10,sticky="ew")
        row_index+=1

        
        self.open_controller_params_button = Button(self, text=f"Edit Controller Parameters", command=lambda: self.open_controller_params_button_function(controller))
        self.open_controller_params_button.grid(row= row_index, column=0, columnspan=1, sticky="ew", padx=20, pady=2)
        row_index +=1

        
        # set zero velocity button
        self.set_velocity_to_zero_button = Button(self, text=f"Set velocity to zero", command=lambda: controller.set_zero_velocity())
        self.set_velocity_to_zero_button.grid(row= row_index, column=0, columnspan=1,  padx=20, pady=10, sticky = "ew" )
        row_index +=1

        # # depth setpoint frame
        # self.depth_setpoint_velocity_frame = LabelFrame(self, text="Set Operating Depth")
        # self.depth_setpoint_velocity_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        # row_index +=1

    
        # velocity setpoint frame
        self.velocity_setpoint_frame= LabelFrame(self, text="Velocity Setpoints")
        self.velocity_setpoint_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        row_index +=1

        # # target depth entry
        # target_depth_label = Label(self.depth_setpoint_velocity_frame, text=f"Target Depth (m)")
        # target_depth_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        
        # target_depth = StringVar()
        # target_depth_entry = Entry(self.depth_setpoint_velocity_frame,textvariable = target_depth, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        # row_index +=1

        # # submit target depth
        # row_index +=1
        # submit_depth_vel_mode_button = Button(self.depth_setpoint_velocity_frame, text=f"Submit", command=lambda: controller.submit_target_depth(target_depth.get()))
        # submit_depth_vel_mode_button.grid(row=row_index, column=1,columnspan=1, padx = 20, pady=10, sticky="w")  

       
        # creating velocity setpoint labels in velocity setpoint frame
        row_index = 0
        vx_label = Label(self.velocity_setpoint_frame, text=f"x velocity (m/s)")
        vx_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        vx = StringVar()
        vx_entry = Entry(self.velocity_setpoint_frame,textvariable = vx, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        
        vy_label = Label(self.velocity_setpoint_frame, text=f"y velocity (m/s)")
        vy_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        vy = StringVar()
        vy_entry = Entry(self.velocity_setpoint_frame,textvariable = vy, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        
        vz_label = Label(self.velocity_setpoint_frame, text=f"z velocity (m/s)")
        vz_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        vz = StringVar()
        vz_entry = Entry(self.velocity_setpoint_frame,textvariable = vz, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        vyaw_label = Label(self.velocity_setpoint_frame, text=f"yaw velocity (deg/s)")
        vyaw_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        vyaw = StringVar()
        vyaw_entry = Entry(self.velocity_setpoint_frame,textvariable = vyaw, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        submit_velocity_button = Button(self.velocity_setpoint_frame, text=f"Submit", command=lambda: controller.submit_velocity_setpoint(vx.get(),vy.get(),vz.get(),vyaw.get()))
        submit_velocity_button.grid(row = row_index, column=1,padx=5, pady=5)   

    def open_controller_params_button_function(self,controller):
        if not controller.open_controller_params_frame:
            controller.open_controller_params_frame = True
            text = "Close Controller Parameters"
        elif controller.open_controller_params_frame:    
            controller.open_controller_params_frame  = False
            text = "Edit Controller Parameters"
            

        self.open_controller_params_button.configure(text=f"{text}")

        controller.open_close_controller_params_window()

class PWMFrame(LabelFrame):
    def __init__(self, parent, controller):
        LabelFrame.__init__(self, parent,text="PWM Mode")

        row_index =0
        self.close_pwm_mode_button = Button(self, text="Disable PWM Mode", command=lambda: controller.show_frame(ControlOptionsPage))
        self.close_pwm_mode_button.grid(row=row_index, column= 0, columnspan=1, padx=20, pady=10,sticky="ew")
        row_index+=1

        # Edit controller params button
        # self.open_controller_params_button = Button(self.velocity_setpoint_mode_frame, text=f"Edit controller gains", command= self.create_controller_params_window)
        # self.open_controller_params_button .grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        # row_index +=1

        

        # set zero PWM button
        self.set_pwm_to_zero_button = Button(self, text=f"Set PWM to zero", command=lambda: controller.set_zero_pwm())
        self.set_pwm_to_zero_button.grid(row= row_index, column=0, columnspan=1,  padx=20, pady=10, sticky = "ew" )
        row_index +=1

        # # depth setpoint frame
        # self.depth_setpoint_pwm_frame = LabelFrame(self, text="Set Operating Depth")
        # self.depth_setpoint_pwm_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        # row_index +=1

    
        # velocity setpoint frame
        self.pwm_setpoint_frame= LabelFrame(self, text="PWM Setpoints")
        self.pwm_setpoint_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        row_index +=1

        # # target depth entry
        # target_depth_label = Label(self.depth_setpoint_pwm_frame, text=f"Target Depth (m)")
        # target_depth_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        # target_depth = StringVar()
        # target_depth_entry = Entry(self.depth_setpoint_pwm_frame,textvariable = target_depth, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        # row_index +=1

        # # submit target depth
        # row_index +=1
        # submit_depth_button = Button(self.depth_setpoint_pwm_frame, text=f"Submit", command=lambda: controller.submit_target_depth(target_depth.get()))
        # submit_depth_button.grid(row=row_index, column=1,columnspan=1, padx = 20, pady=10, sticky="w")  

       
        # creating velocity setpoint labels in velocity setpoint frame
        row_index = 0
        x_pwm_label = Label(self.pwm_setpoint_frame, text=f"x pwm (-1000, 1000)")
        x_pwm_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        x_pwm = StringVar()
        x_pwm_entry = Entry(self.pwm_setpoint_frame,textvariable = x_pwm, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        
        y_pwm_label = Label(self.pwm_setpoint_frame, text=f"y pwm (-1000, 1000)")
        y_pwm_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        y_pwm = StringVar()
        y_pwm_entry = Entry(self.pwm_setpoint_frame,textvariable = y_pwm, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        
        z_pwm_label = Label(self.pwm_setpoint_frame, text=f"z pwm (0, 1000)")
        z_pwm_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        z_pwm = StringVar()
        z_pwm_entry = Entry(self.pwm_setpoint_frame,textvariable = z_pwm, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_pwm_label = Label(self.pwm_setpoint_frame, text=f"yaw pwm (-1000, 1000)")
        yaw_pwm_label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5, sticky="ew")
        yaw_pwm = StringVar()
        yaw_pwm_entry = Entry(self.pwm_setpoint_frame,textvariable = yaw_pwm, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        # submit pwm button
        submit_pwm_button = Button(self.pwm_setpoint_frame, text=f"Submit", command=lambda: controller.submit_pwm_setpoint(x_pwm.get(),y_pwm.get(),z_pwm.get(),yaw_pwm.get()))
        submit_pwm_button.grid(row = row_index, column=1,padx=5, pady=5)   

class WaypointFrame(LabelFrame):
    def __init__(self, parent, controller):
        LabelFrame.__init__(self, parent, text = "Waypoint Mode")
        add_current_pose = False
        
        # get orientation to use along path 

        orientation_options = {"Translate Rotate" : 1,
                                "Rotate Translate" : 2,
                                "Smooth Transition" : 3,
                                "RTR" : 4
                            }

        alg_options = {"Alg 1" : 1,
        "Alg 2" : 2,
        }

        waypoint_type = {"Target" : 1,
        "Intermediate" : 2,
        }



        # variable to hold orientation choice
        orientation_options_choice = StringVar()
        alg_choice = StringVar()
        waypoint_type_choice = StringVar()


        # frames 
        row_index =0
        
        self.close_waypoint_mode_button = Button(self, text="Disable Waypoint Mode", command=lambda: controller.show_frame(ControlOptionsPage))
        self.close_waypoint_mode_button.grid(row=row_index, column= 0, rowspan=1, columnspan =1,padx=20, pady=5,sticky="ew")

        # Orientation Frame
        self.orientation_options_frame = LabelFrame(self, text="Orientation Along Path")
        self.orientation_options_frame.grid(row = row_index, column=1, rowspan=6, columnspan=1, padx=20, pady=10, sticky="new")
        # row_index +=1

        # Alg Choice
        self.algorithm_frame = LabelFrame(self, text="Algorithm")
        self.algorithm_frame.grid(row = row_index, column=2, rowspan=6, columnspan=1, padx=20, pady=10, sticky="new")
        row_index +=1
     
       
        self.hold_pose_button = Button(self, text="Follow Path", command=lambda: self.hold_pose_button_function(controller))
        # var_text = StringVar()
        if controller.hold_pose:
            self.hold_pose_button.configure(text = "Follow Path")
        else:
            self.hold_pose_button.configure(text = "Hold Pose")
        
        self.hold_pose_button.grid(row= row_index, column=0, rowspan=1, columnspan=1, sticky="ew", padx=20, pady=5)
        row_index+=1
         
        self.erase_waypoints_button = Button(self, text="Erase all waypoints", command=lambda: self.erase_waypoints(controller))
        self.erase_waypoints_button.grid(row= row_index, column=0, rowspan=1, columnspan=1, sticky="ew", padx=20, pady=5)
        row_index+=1

        self.open_controller_params_button = Button(self, text=f"Edit Controller Parameters", command=lambda: self.open_controller_params_button_function(controller))
        self.open_controller_params_button.grid(row= row_index, column=0, rowspan=1, columnspan=1, sticky="ew", padx=20, pady=5)
        row_index+=1

        self.open_preset_patterns_button = Button(self, text=f"Open Preset Patterns", command=lambda: self.open_preset_patterns_button_function(controller))
        self.open_preset_patterns_button.grid(row= row_index, column=0, rowspan=1, columnspan=1, sticky="ew", padx=20, pady=5)
        row_index+=1


        # add waypoints frame
        self.add_waypoints_frame = LabelFrame(self, text="Add Waypoints")
        self.add_waypoints_frame.grid(row = row_index, column=0, columnspan=4, padx=20, pady=10, sticky="ew")
        row_index +=1

    

        row_index = 0
        # add waypoints frame 1
        self.add_waypoints_frame_1 = LabelFrame(self.add_waypoints_frame, text="Add a waypoint relative to \nthe current pose")
        self.add_waypoints_frame_1.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
      
        # add waypoints frame 2
        self.add_waypoints_frame_2 = LabelFrame(self.add_waypoints_frame, text="Add a waypoint relative to \nthe last waypoint added")
        self.add_waypoints_frame_2.grid(row = row_index, column=1, columnspan=1, padx=20, pady=10, sticky="ew")
        # row_index +=1

       
        # add waypoints frame 3
        self.add_waypoints_frame_3 = LabelFrame(self.add_waypoints_frame, text="Add a waypoint relative to \nNED frame")
        self.add_waypoints_frame_3.grid(row = row_index, column=2, columnspan=1, padx=20, pady=10, sticky="ew")
       
        # ready for gps if needed
        
        # add waypoints frame 4
        self.add_waypoints_frame_4 = LabelFrame(self.add_waypoints_frame, text="Add a GPS waypoint")
        self.add_waypoints_frame_4.grid(row = row_index, column=3, columnspan=1, padx=20, pady=10, sticky="ew")

        if add_current_pose:
            # add waypoints frame 3
            self.add_waypoints_frame_5 = LabelFrame(self.add_waypoints_frame, text="Test current pose")
            self.add_waypoints_frame_5.grid(row = row_index+1, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
       
        

        row_index = 0
        # waypoint relative to current pose
        x_label_1 = Label(self.add_waypoints_frame_1, text="X:")
        x_label_1.grid(row=row_index, column=0, padx=5, pady=5)
        x1 = StringVar()
        x1_entry  = Entry(self.add_waypoints_frame_1,textvariable = x1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        y_label_1 = Label(self.add_waypoints_frame_1, text="Y:")
        y_label_1.grid(row=row_index, column=0, padx=5, pady=5)
        y1 = StringVar()
        y1_entry = Entry(self.add_waypoints_frame_1,textvariable = y1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        z_label_1 = Label(self.add_waypoints_frame_1, text="Z:")
        z_label_1.grid(row=row_index, column=0, padx=5, pady=5)
        z1 = StringVar()
        z1_entry = Entry(self.add_waypoints_frame_1,textvariable = z1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_label_1 = Label(self.add_waypoints_frame_1, text="yaw (deg):")
        yaw_label_1.grid(row=row_index, column=0, padx=5, pady=5)
        yaw1 = StringVar()
        yaw1_entry = Entry(self.add_waypoints_frame_1,textvariable = yaw1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
    
        self.visualize_waypoint_button_1 = Button(self.add_waypoints_frame_1, text=f"Visualize", command=lambda: controller.visualize_waypoint_1(x1.get(),y1.get(),z1.get(),yaw1.get()))
        self.visualize_waypoint_button_1 .grid(row = row_index, column=0,padx=5, pady=5)  
        self.submit_waypoint_button_1 = Button(self.add_waypoints_frame_1, text=f"Submit", command=lambda: controller.submit_waypoint_1(x1.get(),y1.get(),z1.get(),yaw1.get(), orientation_options_choice.get(), alg_choice.get()))
        self.submit_waypoint_button_1 .grid(row = row_index, column=1,padx=5, pady=5) 
        self.submit_waypoint_button_1.config(state=DISABLED)

            
              



        row_index = 0
        # waypoint relative to last waypoint
        x_label_2 = Label(self.add_waypoints_frame_2, text="X:")
        x_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        x2 = StringVar()
        x2_entry  = Entry(self.add_waypoints_frame_2,textvariable = x2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        y_label_2 = Label(self.add_waypoints_frame_2, text="Y:")
        y_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        y2 = StringVar()
        y2_entry = Entry(self.add_waypoints_frame_2,textvariable = y2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        z_label_2 = Label(self.add_waypoints_frame_2, text="Z:")
        z_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        z2 = StringVar()
        z2_entry = Entry(self.add_waypoints_frame_2,textvariable = z2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_label_2 = Label(self.add_waypoints_frame_2, text="yaw (deg):")
        yaw_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        yaw2 = StringVar()
        yaw2_entry = Entry(self.add_waypoints_frame_2,textvariable = yaw2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
    
        visualize_waypoint_button_2 = Button(self.add_waypoints_frame_2, text=f"Visualize", command=lambda: controller.visualize_waypoint_2(x2.get(),y2.get(),z2.get(),yaw2.get()))
        visualize_waypoint_button_2 .grid(row = row_index, column=0,padx=5, pady=5)  
        submit_waypoint_button_2 = Button(self.add_waypoints_frame_2, text=f"Submit", command=lambda: controller.submit_waypoint_2(x2.get(),y2.get(),z2.get(),yaw2.get(),orientation_options_choice.get(), alg_choice.get()))
        submit_waypoint_button_2 .grid(row = row_index, column=1,padx=5, pady=5)   

        
        row_index = 0
        # waypoint relative to NED
        x_label_3 = Label(self.add_waypoints_frame_3, text="X:")
        x_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        x3 = StringVar()
        x3_entry  = Entry(self.add_waypoints_frame_3,textvariable = x3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        y_label_3 = Label(self.add_waypoints_frame_3, text="Y:")
        y_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        y3 = StringVar()
        y3_entry = Entry(self.add_waypoints_frame_3,textvariable = y3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        z_label_3 = Label(self.add_waypoints_frame_3, text="Z:")
        z_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        z3 = StringVar()
        z3_entry = Entry(self.add_waypoints_frame_3,textvariable = z3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_label_3 = Label(self.add_waypoints_frame_3, text="yaw (deg):")
        yaw_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        yaw3 = StringVar()
        yaw3_entry = Entry(self.add_waypoints_frame_3,textvariable = yaw3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
    
        visualize_waypoint_button_3 = Button(self.add_waypoints_frame_3, text=f"Visualize", command=lambda: controller.visualize_waypoint_3(x3.get(),y3.get(),z3.get(),yaw3.get()))
        visualize_waypoint_button_3 .grid(row = row_index, column=0,padx=5, pady=5)  

        submit_waypoint_button_3 = Button(self.add_waypoints_frame_3, text=f"Submit", command=lambda: controller.submit_waypoint_3(x3.get(),y3.get(),z3.get(),yaw3.get(), orientation_options_choice.get(), alg_choice.get()))
        submit_waypoint_button_3 .grid(row = row_index, column=1,padx=5, pady=5)   

        row_index = 0
        # waypoint relative to last waypoint
        x_label_2 = Label(self.add_waypoints_frame_2, text="X:")
        x_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        x2 = StringVar()
        x2_entry  = Entry(self.add_waypoints_frame_2,textvariable = x2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        y_label_2 = Label(self.add_waypoints_frame_2, text="Y:")
        y_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        y2 = StringVar()
        y2_entry = Entry(self.add_waypoints_frame_2,textvariable = y2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        z_label_2 = Label(self.add_waypoints_frame_2, text="Z:")
        z_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        z2 = StringVar()
        z2_entry = Entry(self.add_waypoints_frame_2,textvariable = z2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_label_2 = Label(self.add_waypoints_frame_2, text="yaw (deg):")
        yaw_label_2.grid(row=row_index, column=0, padx=5, pady=5)
        yaw2 = StringVar()
        yaw2_entry = Entry(self.add_waypoints_frame_2,textvariable = yaw2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
    
        visualize_waypoint_button_2 = Button(self.add_waypoints_frame_2, text=f"Visualize", command=lambda: controller.visualize_waypoint_2(x2.get(),y2.get(),z2.get(),yaw2.get()))
        visualize_waypoint_button_2 .grid(row = row_index, column=0,padx=5, pady=5)  
        submit_waypoint_button_2 = Button(self.add_waypoints_frame_2, text=f"Submit", command=lambda: controller.submit_waypoint_2(x2.get(),y2.get(),z2.get(),yaw2.get(),orientation_options_choice.get(), alg_choice.get()))
        submit_waypoint_button_2 .grid(row = row_index, column=1,padx=5, pady=5)   

        
        row_index = 0
        # waypoint relative to NED
        x_label_3 = Label(self.add_waypoints_frame_3, text="X:")
        x_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        x3 = StringVar()
        x3_entry  = Entry(self.add_waypoints_frame_3,textvariable = x3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        y_label_3 = Label(self.add_waypoints_frame_3, text="Y:")
        y_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        y3 = StringVar()
        y3_entry = Entry(self.add_waypoints_frame_3,textvariable = y3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        z_label_3 = Label(self.add_waypoints_frame_3, text="Z:")
        z_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        z3 = StringVar()
        z3_entry = Entry(self.add_waypoints_frame_3,textvariable = z3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_label_3 = Label(self.add_waypoints_frame_3, text="yaw (deg):")
        yaw_label_3.grid(row=row_index, column=0, padx=5, pady=5)
        yaw3 = StringVar()
        yaw3_entry = Entry(self.add_waypoints_frame_3,textvariable = yaw3, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
    
        visualize_waypoint_button_3 = Button(self.add_waypoints_frame_3, text=f"Visualize", command=lambda: controller.visualize_waypoint_3(x3.get(),y3.get(),z3.get(),yaw3.get()))
        visualize_waypoint_button_3 .grid(row = row_index, column=0,padx=5, pady=5)  

        submit_waypoint_button_3 = Button(self.add_waypoints_frame_3, text=f"Submit", command=lambda: controller.submit_waypoint_3(x3.get(),y3.get(),z3.get(),yaw3.get(), orientation_options_choice.get(), alg_choice.get()))
        submit_waypoint_button_3 .grid(row = row_index, column=1,padx=5, pady=5)   

        # gps waypoint
        lat_label = Label(self.add_waypoints_frame_4, text="Latitude")
        lat_label.grid(row=row_index, column=0, padx=5, pady=5)
        latitude = StringVar()
        lat_entry  = Entry(self.add_waypoints_frame_4,textvariable = latitude, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        long_label = Label(self.add_waypoints_frame_4, text="Longitude")
        long_label.grid(row=row_index, column=0, padx=5, pady=5)
        longitude = StringVar()
        long_entry  = Entry(self.add_waypoints_frame_4,textvariable = longitude, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        depth_label = Label(self.add_waypoints_frame_4, text="Depth")
        depth_label.grid(row=row_index, column=0, padx=5, pady=5)
        depth = StringVar()
        depth_entry  = Entry(self.add_waypoints_frame_4,textvariable = depth, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_label_4= Label(self.add_waypoints_frame_4, text="yaw (deg):")
        yaw_label_4.grid(row=row_index, column=0, padx=5, pady=5)
        yaw4 = StringVar()
        yaw4_entry = Entry(self.add_waypoints_frame_4,textvariable = yaw4, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
    
        visualize_waypoint_button_4 = Button(self.add_waypoints_frame_4, text=f"Visualize", command=lambda: controller.visualize_waypoint_4(latitude.get(), longitude.get(), depth.get(),yaw4.get()))
        visualize_waypoint_button_4 .grid(row = row_index, column=0,padx=5, pady=5)  

        submit_waypoint_button_4 = Button(self.add_waypoints_frame_4, text=f"Submit", command=lambda: controller.submit_waypoint_4(latitude.get(), longitude.get(), depth.get(),yaw4.get(), orientation_options_choice.get(), alg_choice.get()))
        submit_waypoint_button_4 .grid(row = row_index, column=1,padx=5, pady=5)   

        if add_current_pose:
            # waypoint relative to NED
            x_label_5 = Label(self.add_waypoints_frame_5, text="X:")
            x_label_5.grid(row=row_index, column=0, padx=5, pady=5)
            x5 = StringVar()
            x5_entry  = Entry(self.add_waypoints_frame_5,textvariable = x5, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
            row_index +=1

            y_label_5 = Label(self.add_waypoints_frame_5, text="Y:")
            y_label_5.grid(row=row_index, column=0, padx=5, pady=5)
            y5 = StringVar()
            y5_entry = Entry(self.add_waypoints_frame_5,textvariable = y5, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
            row_index +=1

            z_label_5 = Label(self.add_waypoints_frame_5, text="Z:")
            z_label_5.grid(row=row_index, column=0, padx=5, pady=5)
            z5 = StringVar()
            z5_entry = Entry(self.add_waypoints_frame_5,textvariable = z5, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
            row_index +=1

            yaw_label_5 = Label(self.add_waypoints_frame_5, text="yaw (deg):")
            yaw_label_5.grid(row=row_index, column=0, padx=5, pady=5)
            yaw5 = StringVar()
            yaw5_entry = Entry(self.add_waypoints_frame_5,textvariable = yaw5, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
            row_index +=1
        
            # visualize_waypoint_button_5 = Button(self.add_waypoints_frame_5, text=f"Visualize", command=lambda: controller.visualize_current_pose(x5.get(),y5.get(),z5.get(),yaw5.get()))
            # visualize_waypoint_button_5 .grid(row = row_index, column=0,padx=5, pady=5)  

            submit_waypoint_button_5 = Button(self.add_waypoints_frame_5, text=f"Submit", command=lambda: controller.submit_current_pose(x5.get(),y5.get(),z5.get(),yaw5.get(), orientation_options_choice.get(), alg_choice.get()))
            submit_waypoint_button_5 .grid(row = row_index, column=1,padx=5, pady=5)   

            


        # get orientation to use along path 

        for (text, value) in orientation_options.items():
        
            orientation_options_button= tk.Radiobutton(self.orientation_options_frame, text = text, variable =  orientation_options_choice, 
                        value = value, indicatoron=0, padx=10,pady=5)
            orientation_options_button.grid(row = value-1, column = 0, padx=10, pady=5, columnspan =1, sticky='ew')


        # get Algorithm to use

        for (text, value) in alg_options.items():
        
            alg_choice_button= tk.Radiobutton(self.algorithm_frame, text = text, variable =  alg_choice, 
                        value = value, indicatoron=0, padx=10,pady=5)
            alg_choice_button.grid(row = value-1, column = 0, padx=10, pady=5, columnspan =1, sticky='ew')


    def path_generation_button(self, indicator):
        # self.path_type_indicator = indicator
        
        rospy.loginfo(f"path indicator: {indicator}")
    
    def orientation_generation_button(self, indicator):
        # self.orientation_type_indicator_type_indicator = indicator
        
        rospy.loginfo(f"orientation indicator: {indicator}")

    def erase_waypoints(self,controller):
        self.hold_pose_button.configure(text="Follow Path")
        controller.erase_waypoints()
            
    def hold_pose_button_function(self,controller):
        # logic is  backwards compared to what would be expected because were changing it before updating the variable in the other class
        if controller.hold_pose:
            if controller.goal_waypoints.poses:
              
                self.hold_pose_button.configure(text="Hold Pose")
                controller.hold_pose_button_function()
            else: 
                rospy.logerr("No waypoints provided")
            
        elif not controller.hold_pose:
            
                self.hold_pose_button.configure(text="Follow Path")
                controller.hold_pose_button_function()

    def open_controller_params_button_function(self,controller):
        if not controller.open_controller_params_frame:
            controller.open_controller_params_frame = True
            text = "Close Controller Parameters"
        elif controller.open_controller_params_frame:    
            controller.open_controller_params_frame  = False
            text = "Edit Controller Parameters"
            

        self.open_controller_params_button.configure(text=f"{text}")

        controller.open_close_controller_params_window()

    def open_preset_patterns_button_function(self,controller):
        if not controller.open_preset_patterns_frame:
            controller.open_preset_patterns_frame = True
            text = "Close Preset Patterns "
        elif controller.open_preset_patterns_frame:    
            controller.open_preset_patterns_frame  = False
            text = "Open Preset Patterns"
            

        self.open_preset_patterns_button.configure(text=f"{text}")

        controller.open_close_preset_patterns_window()


# frames that open in new Windows
class ControllerParamFrame(Toplevel):
    def __init__(self, parent, controller):
        
        super().__init__(master = parent)
        self.title("Controller Parameters")
        # Add a label widget
        row_index = 0
        # Button to return to the main window
        self.controller_params_window_back_button = Button(self, text="Close Window", command=lambda: self.close(controller))
        self.controller_params_window_back_button.grid(row = row_index, column=0, columnspan=2, sticky="ew", padx=20, pady=20)

        row_index +=1

        # Position Controller Parameters
        self.position_controller_params = LabelFrame(self, text="Position Controller Gains")
        self.position_controller_params.grid(row =row_index, column=0, columnspan=1, padx =20, pady=20, sticky="ew")
        # row_index+=1
        # Position Controller Parameters
        self.velocity_controller_params  = LabelFrame(self, text="Velocity Controller Gains")
        self.velocity_controller_params.grid(row =row_index, column=1, columnspan=1, padx =20, pady=20, sticky="ew")
        row_index +=1

        # Saturation Parameters
        self.saturation_params = LabelFrame(self, text="Saturation Parameters")
        self.saturation_params.grid(row = row_index, column=0, columnspan=2, padx = 20, pady=20, sticky="ew")
        row_index +=1


        # Saturation Parameters
        self.path_following_params_frame = LabelFrame(self, text="Path Following Parameters")
        self.path_following_params_frame.grid(row = row_index, column=0, columnspan=2, padx = 20, pady=20, sticky="ew")
       
        row_index +=1
        
    

        # creating velocity setpoint labels in velocity setpoint frame
        row_index = 0

        label = Label(self.position_controller_params , text=f"kp")
        label.grid(row=row_index, column=1,columnspan=1, padx=5, pady=5)
        label = Label(self.position_controller_params , text=f"kd")
        label.grid(row=row_index, column=2,columnspan=1, padx=5, pady=5)
        label = Label(self.position_controller_params , text=f"ki")
        label.grid(row=row_index, column=3,columnspan=1, padx=5, pady=5)
        row_index += 1



        # X ENTRY
        label = Label(self.position_controller_params , text=f"x")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpx1 = StringVar()
        kpx1_entry = Entry(self.position_controller_params ,textvariable = kpx1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdx1 = StringVar()
        kdx1_entry = Entry(self.position_controller_params ,textvariable = kdx1, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kix1 = StringVar()
        kix1_entry = Entry(self.position_controller_params ,textvariable = kix1, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1



        # Y ENTRY
        label = Label(self.position_controller_params , text=f"y")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpy1 = StringVar()
        kpy1_entry = Entry(self.position_controller_params ,textvariable = kpy1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdy1 = StringVar()
        kdy1_entry = Entry(self.position_controller_params ,textvariable = kdy1, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kiy1 = StringVar()
        kiy1_entry = Entry(self.position_controller_params ,textvariable = kiy1, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1

        # Z ENTRY
        label = Label(self.position_controller_params , text=f"z")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpz1 = StringVar()
        kpyz_entry = Entry(self.position_controller_params ,textvariable = kpz1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdz1 = StringVar()
        kdz1_entry = Entry(self.position_controller_params ,textvariable = kdz1, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kiz1 = StringVar()
        kiz1_entry = Entry(self.position_controller_params ,textvariable = kiz1, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1

        # YAW ENTRY
        label = Label(self.position_controller_params , text=f"yaw")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpyaw1 = StringVar()
        kpyaw1_entry = Entry(self.position_controller_params ,textvariable = kpyaw1, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdyaw1 = StringVar()
        kdyaw1_entry = Entry(self.position_controller_params ,textvariable = kdyaw1, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kiyaw1 = StringVar()
        kiyaw1_entry = Entry(self.position_controller_params ,textvariable = kiyaw1, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1

        # Button to return to the main window
        self.submit_position_gains_button = Button(self.position_controller_params, text="Submit", command=lambda: controller.submit_controller_gains(1,kpx1.get(), kdx1.get(), kix1.get(),
             kpy1.get(), kdy1.get(), kiy1.get(),
             kpz1.get(), kdz1.get(), kiz1.get(),
             kpyaw1.get(), kdyaw1.get(), kiyaw1.get()))
        self.submit_position_gains_button.grid(row = row_index, column=1,columnspan=3, sticky="ew", padx=5, pady=10)
        
        row_index+=1

        

        # creating velocity setpoint labels in velocity setpoint frame
        row_index = 0

        label = Label(self.velocity_controller_params  , text=f"kp")
        label.grid(row=row_index, column=1,columnspan=1, padx=5, pady=5)
        label = Label(self.velocity_controller_params  , text=f"kd")
        label.grid(row=row_index, column=2,columnspan=1, padx=5, pady=5)
        label = Label(self.velocity_controller_params  , text=f"ki")
        label.grid(row=row_index, column=3,columnspan=1, padx=5, pady=5)
        row_index += 1



        # X ENTRY
        label = Label(self.velocity_controller_params  , text=f"x")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpx2 = StringVar()
        kpx2_entry = Entry(self.velocity_controller_params  ,textvariable = kpx2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdx2 = StringVar()
        kdx2_entry = Entry(self.velocity_controller_params  ,textvariable = kdx2, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kix2 = StringVar()
        kix2_entry = Entry(self.velocity_controller_params  ,textvariable = kix2, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1



        # Y ENTRY
        label = Label(self.velocity_controller_params  , text=f"y")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpy2 = StringVar()
        kpy2_entry = Entry(self.velocity_controller_params  ,textvariable = kpy2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdy2 = StringVar()
        kdy2_entry = Entry(self.velocity_controller_params  ,textvariable = kdy2, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kiy2 = StringVar()
        kiy2_entry = Entry(self.velocity_controller_params  ,textvariable = kiy2, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1

        # Z ENTRY
        label = Label(self.velocity_controller_params  , text=f"z")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpz2 = StringVar()
        kpyz_entry = Entry(self.velocity_controller_params  ,textvariable = kpz2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdz2 = StringVar()
        kdz2_entry = Entry(self.velocity_controller_params  ,textvariable = kdz2, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kiz2 = StringVar()
        kiz2_entry = Entry(self.velocity_controller_params  ,textvariable = kiz2, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1

        # YAW ENTRY
        label = Label(self.velocity_controller_params  , text=f"yaw")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)

        kpyaw2 = StringVar()
        kpyaw2_entry = Entry(self.velocity_controller_params  ,textvariable = kpyaw2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
    
        kdyaw2 = StringVar()
        kdyaw2_entry = Entry(self.velocity_controller_params  ,textvariable = kdyaw2, width=10).grid(row=row_index, column=2,padx=5, pady=5, sticky="ew")
        
        kiyaw2 = StringVar()
        kiyaw2_entry = Entry(self.velocity_controller_params  ,textvariable = kiyaw2, width=10).grid(row=row_index, column=3,padx=5, pady=5, sticky="ew")
        
        row_index +=1

        # Button to return to the main window
        self.submit_position_gains_button = Button(self.velocity_controller_params , text="Submit", command=lambda: controller.submit_controller_gains(2, kpx2.get(), kdx2.get(), kix2.get(),
             kpy2.get(), kdy2.get(), kiy2.get(),
             kpz2.get(), kdz2.get(), kiz2.get(),
             kpyaw2.get(), kdyaw2.get(), kiyaw2.get()))
        self.submit_position_gains_button.grid(row = row_index, column=1,columnspan=3, sticky="ew", padx=5, pady=10)
    

        #  saturation parameters
        row_index = 0

        # max lin velocity 
        label = Label(self.saturation_params, text=f"max linear velocity (m/s)")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        max_lin_velocity = StringVar()
        max_lin_velocity_entry = Entry(self.saturation_params, textvariable = max_lin_velocity, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        self.submit_linear_velocity_button = Button(self.saturation_params, text=f"Submit", command=lambda: controller.submit_max_linear_velocity(max_lin_velocity.get()))
        self.submit_linear_velocity_button.grid(row= row_index, column=2,  padx=5, pady=10)
        row_index +=1

        label = Label(self.saturation_params, text=f"max angular velocity (deg/s)")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        max_ang_velocity = StringVar()
        max_ang_velocity_entry = Entry(self.saturation_params, textvariable = max_ang_velocity, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        self.submit_angular_velocity_button = Button(self.saturation_params, text=f"Submit", command=lambda: controller.submit_max_angular_velocity(max_ang_velocity.get()))
        self.submit_angular_velocity_button.grid(row= row_index, column=2, padx=5, pady=10)
        row_index +=1       

        label = Label(self.saturation_params, text=f"max x, y, yaw pwm (0 - 1000)")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        max_pwm= StringVar()
        max_pwm_entry = Entry(self.saturation_params, textvariable = max_pwm, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        self.submit_x_y_pwm_button = Button(self.saturation_params, text=f"Submit", command=lambda: controller.submit_max_pwm(max_pwm.get()))
        self.submit_x_y_pwm_button.grid(row= row_index, column=2, padx=5, pady=10)
        row_index +=1       

        #  Path following Params
        row_index = 0

        # position Threshold
        label = Label(self.path_following_params_frame, text=f"Position Threshold (m)")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        pos_threshold = StringVar()
        pos_threshold_entry = Entry(self.path_following_params_frame, textvariable = pos_threshold, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        submit_pos_threshold_button = Button(self.path_following_params_frame, text=f"Submit", command=lambda: controller.submit_pos_threshold(pos_threshold.get()))
        submit_pos_threshold_button.grid(row= row_index, column=2,  padx=5, pady=10)
        row_index +=1

        # orientation threshold
        label = Label(self.path_following_params_frame, text=f"Orientation Threshold (deg)")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        orientation_threshold = StringVar()
        orientation_threshold_entry = Entry(self.path_following_params_frame, textvariable = orientation_threshold, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        submit_orientation_threshold_button = Button(self.path_following_params_frame, text=f"Submit", command=lambda: controller.submit_orientation_threshold(orientation_threshold.get()))
        submit_orientation_threshold_button.grid(row= row_index, column=2,  padx=5, pady=10)
        row_index +=1

        # lookahead distance
        label = Label(self.path_following_params_frame, text=f"Lookahead Distance (m)")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        lookahead_distance = StringVar()
        lookahead_distance_entry = Entry(self.path_following_params_frame, textvariable = lookahead_distance, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        submit_lookahead_distance_button = Button(self.path_following_params_frame, text=f"Submit", command=lambda: controller.submit_lookahead_distance(lookahead_distance.get()))
        submit_lookahead_distance_button.grid(row= row_index, column=2,  padx=5, pady=10)
        row_index +=1

        # lookahead distance past waypoint
        label = Label(self.path_following_params_frame, text=f"Pure Pursuit Switch (m)")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        lookahead_distance2 = StringVar()
        lookahead_distance2_entry = Entry(self.path_following_params_frame, textvariable = lookahead_distance2, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        submit_lookahead_distance2_button = Button(self.path_following_params_frame, text=f"Submit", command=lambda: controller.submit_lookahead_distance2(lookahead_distance2.get()))
        submit_lookahead_distance2_button.grid(row= row_index, column=2,  padx=5, pady=10)
        row_index +=1

        # Choose path following alg
        label = Label(self.path_following_params_frame, text=f"Path following algorithm")
        label.grid(row=row_index, column=0,columnspan=1, padx=5, pady=5)
        algorithm = StringVar()
        algorithm_entry = Entry(self.path_following_params_frame, textvariable = algorithm, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        submit_algorithm_button = Button(self.path_following_params_frame, text=f"Submit", command=lambda: controller.submit_algorithm(algorithm.get()))
        submit_algorithm_button.grid(row= row_index, column=2,  padx=5, pady=10)
        row_index +=1
  


    def close(self,controller):
        controller.open_controller_params_frame = False
        controller.open_close_controller_params_window()

class PresetPatternsFrame(Toplevel):
    def __init__(self, parent, controller):

        super().__init__(master = parent)
        self.title("Preset Patterns")
       
        row_index = 0

        # Button to return to the main window
        self.preset_patterns_frame_back_button = Button(self, text="Close Window", command=lambda: self.close(controller))
        self.preset_patterns_frame_back_button.grid(row = row_index, column=0, columnspan=1, sticky="ew", padx=20, pady=20)
        row_index +=1

        # Preset patterns frames
        self.preset_patterns_frame = LabelFrame(self, text="Preset Patterns")
        self.preset_patterns_frame.grid(row = row_index, column=0, rowspan=10, columnspan=1, padx=20, pady=10, sticky="new")
        row_index +=1


         
        relative_options = {"current pose" : 1,
                        "last waypoint" :  2,
                        }
        rotate_direction_options = {"clockwise" : 1,
                                    "counterclockwise" :2
                                    }

        relative_option_button_text = StringVar()
        rotate_directions_button_text1 = StringVar()
        rotate_directions_button_text2 = StringVar()
        rotate_directions_button_text3 = StringVar()
        rotate_directions_button_text4 = StringVar()


        orientation_options = {"Translate Rotate" : 1,
                                "Rotate Translate" : 2,
                                "Smooth Transition" : 3,
                                "RTR" : 4
                            }

        alg_options = {"Alg 1" : 1,
                "Alg 2" : 2,
                }

        
        orientation_options_choice2 = StringVar()
        alg_choice2 = StringVar()


        row_index =0
    
        # Orientation Frame
        self.orientation_options_frame2 = LabelFrame(self.preset_patterns_frame, text="Orientation Along Path")
        # self.orientation_options_frame2.grid(row = 3, column=0, rowspan=6, columnspan=2, padx=20, pady=10, sticky="new")
        self.orientation_options_frame2.grid(row = 3, column=0, columnspan=2, padx=20, pady=10, sticky="new")
         # get orientation to use along path 

        for (text, value) in orientation_options.items():
        
            orientation_options_button2= tk.Radiobutton(self.orientation_options_frame2, text = text, variable =  orientation_options_choice2, 
                        value = value, indicatoron=0, padx=10,pady=5)
            # orientation_options_button.grid(row = value, column = 0, padx=5, pady=5,columnspan =2,sticky='ew')
            orientation_options_button2.grid(row = value-1, column = 0, padx=10, pady=5, columnspan =1, sticky='ew')


        # algorithm frame
        self.algorithm_frame2 = LabelFrame(self.preset_patterns_frame, text="Algorithm")
        self.algorithm_frame2.grid(row = 3, column=2, rowspan=6, columnspan=2, padx=20, pady=10, sticky="new")

        # alg_choice
        
        for (text, value) in alg_options.items():
        
            alg_choice_button2= tk.Radiobutton(self.algorithm_frame2, text = text, variable =  alg_choice2, 
                        value = value, indicatoron=0, padx=10, pady=5)
            alg_choice_button2.grid(row = value-1, column = 0, padx=10, pady=5, sticky='ew')
            

       
        # preset patterns frames
        row_index = 0 
        start_point_label = Label(self.preset_patterns_frame, text="Specify start pose relative to: ")
        start_point_label.grid(row=0, column=0, columnspan=2, padx=20, pady=10, sticky='ew')

        for (text, value) in relative_options.items():
        
            relative_option_button = tk.Radiobutton(self.preset_patterns_frame, text = text, variable =  relative_option_button_text, 
                        value = value, indicatoron=0, padx=10,pady=5)
        
            
            # relative_option_button.grid(row = 0, column=value, padx=5, pady=5, sticky="ew")
            relative_option_button.grid(row = 1, column=value-1, padx=5, pady=5, sticky="ew")


        row_index = 4
        # self.square_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Square Pattern")
        # self.square_pattern_frame.grid(row = row_index, column=0, rowspan=1, columnspan=1, padx=20, pady=10, sticky="new")
        # # row_index +=1
        
        # self.lawnmower_frame = LabelFrame(self.preset_patterns_frame , text="Lawnmower Pattern")
        # self.lawnmower_frame.grid(row = row_index, column=1, rowspan=1, columnspan=1, padx=20, pady=10, sticky="new")
        # row_index +=1

    

        self.circle_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Orbit Pattern")
        self.circle_pattern_frame.grid(row = row_index, column=0, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
        # row_index +=1

        # self.lawnmower_frame = LabelFrame(self.preset_patterns_frame , text="Lawnmower Pattern")
        # self.lawnmower_frame.grid(row = row_index, column=0, rowspan=1, columnspan=1, padx=20, pady=10, sticky="new")
        # row_index +=1

        self.arc_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Arc Pattern")
        self.arc_pattern_frame.grid(row = row_index, column=2, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
        row_index +=1


        
        self.lawnmower_frame = LabelFrame(self.preset_patterns_frame , text="Lawnmower Pattern")
        self.lawnmower_frame.grid(row = row_index, column=0, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
        # row_index +=1
        

        self.square_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Square Pattern")
        self.square_pattern_frame.grid(row = row_index, column=2, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
        row_index +=1

    



        # square pattern frame
        row_index = 0
        # choose rotation direction
    
        for (text, value) in rotate_direction_options.items():
        
            rotation_direction_option_button1 = tk.Radiobutton(self.square_pattern_frame, text = text, variable =  rotate_directions_button_text1, 
                        value = value, indicatoron=0, padx=10,pady=5)
            rotation_direction_option_button1.grid(row = row_index, column = value-1, padx=5, pady=5, sticky="ew")

        row_index +=1
        square_size_label = Label(self.square_pattern_frame, text="Length (m)")
        square_size_label.grid(row=row_index, column=0, padx=5, pady=5)
        square_size = StringVar()
        square_size_entry = Entry(self.square_pattern_frame,textvariable = square_size, width=10).grid(row=row_index, column=1,columnspan=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        visualize_generate_square_path_button = Button(self.square_pattern_frame, text=f"Visualize", command=lambda: controller.visualize_square(relative_option_button_text.get(), square_size.get(),rotate_directions_button_text1.get(),orientation_options_choice2.get()))
        visualize_generate_square_path_button.grid(row = row_index, column=0, columnspan =1, padx=5, pady=5, sticky = 'ew')   

        self.submit_generate_square_path_button = Button(self.square_pattern_frame, text=f"Submit", command=lambda: controller.generate_square(relative_option_button_text.get(), square_size.get(),rotate_directions_button_text1.get(),orientation_options_choice2.get(), alg_choice2.get()))
        self.submit_generate_square_path_button.grid(row = row_index, column=1, columnspan =1, padx=5, pady=5, sticky = 'ew')   
        self.submit_generate_square_path_button.configure(state=DISABLED)
        
        # circle pattern frame
        row_index = 0

        # rotation direction choice
        for (text, value) in rotate_direction_options.items():
        
            rotation_direction_option_button2 = tk.Radiobutton(self.circle_pattern_frame , text = text, variable =  rotate_directions_button_text2, 
                        value = value, indicatoron=0, padx=10,pady=5)
            rotation_direction_option_button2.grid(row = row_index, column = value-1, padx=5, pady=5, sticky="ew")

        row_index +=1
        
        # specifying parameters
        circle_radius_label = Label(self.circle_pattern_frame, text="Radius (m)")
        circle_radius_label.grid(row= row_index, column=0, padx=5, pady=5)
        radius = StringVar()
        radius_entry = Entry(self.circle_pattern_frame,textvariable = radius, width=10).grid(row= row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        angle_label = Label(self.circle_pattern_frame, text="Angle (deg)")
        angle_label.grid(row= row_index, column=0, padx=5, pady=5)
        angle = StringVar()
        angle_entry = Entry(self.circle_pattern_frame,textvariable = angle, width=10).grid(row= row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        visualize_generate_orbit_path_button = Button(self.circle_pattern_frame, text=f"Visualize", command=lambda: controller.visualize_orbit(relative_option_button_text.get(), radius.get(), angle.get(), rotate_directions_button_text2.get(), orientation_options["Smooth Transition"]))
        visualize_generate_orbit_path_button.grid(row = row_index, column=0,columnspan=1, padx=5, pady=5, sticky='ew')   
        
        self.submit_generate_orbit_path_button = Button(self.circle_pattern_frame, text=f"Submit", command=lambda: controller.generate_orbit(relative_option_button_text.get(), radius.get(), angle.get(), rotate_directions_button_text2.get(), orientation_options["Smooth Transition"], alg_choice2.get()))
        self.submit_generate_orbit_path_button.grid(row = row_index, column=1,columnspan=1, padx=5, pady=5, sticky='ew')   
        self.submit_generate_orbit_path_button.configure(state=DISABLED)

        # Lawnmower pattern frame
        row_index = 0
        # rotation direction choice
        for (text, value) in rotate_direction_options.items():
        
            rotation_direction_option_button3 = tk.Radiobutton(self.lawnmower_frame, text = text, variable =  rotate_directions_button_text3, 
                        value = value, indicatoron=0, padx=10,pady=5)
            rotation_direction_option_button3.grid(row = row_index, column = value-1, padx=5, pady=5, sticky="ew")

        row_index +=1
        
        # specifying parameters
        leg_length_label = Label(self.lawnmower_frame, text="leg Length (m)")
        leg_length_label.grid(row= row_index, column=0, padx=5, pady=5)
        leg_length = StringVar()
        leg_length_entry = Entry(self.lawnmower_frame,textvariable = leg_length, width=10).grid(row= row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        leg_spacing_label = Label(self.lawnmower_frame, text="Leg Spacing (m)")
        leg_spacing_label.grid(row= row_index, column=0, padx=5, pady=5)
        leg_spacing = StringVar()
        angle_entry = Entry(self.lawnmower_frame,textvariable = leg_spacing, width=10).grid(row= row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        number_of_legs_label = Label(self.lawnmower_frame, text="Number of Legs")
        number_of_legs_label.grid(row= row_index, column=0, padx=5, pady=5)
        n_legs = StringVar()
        n_legs_entry = Entry(self.lawnmower_frame,textvariable = n_legs, width=10).grid(row= row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        visualize_generate_lawnmower_button = Button(self.lawnmower_frame, text=f"Visualize", command=lambda: controller.visualize_lawnmower(relative_option_button_text.get(), leg_length.get(), leg_spacing.get(), n_legs.get(),rotate_directions_button_text3.get(), orientation_options["Translate Rotate"]))
        visualize_generate_lawnmower_button.grid(row = row_index, column=0,columnspan=1, padx=5, pady=5, sticky='ew')
        # row_index+=1

        self.submit_generate_lawnmower_button = Button(self.lawnmower_frame, text=f"Submit", command=lambda: controller.generate_lawnmower(relative_option_button_text.get(), leg_length.get(), leg_spacing.get(), n_legs.get(),rotate_directions_button_text3.get(), orientation_options["Translate Rotate"], alg_choice2.get()))
        self.submit_generate_lawnmower_button.grid(row = row_index, column=1,columnspan=1, padx=5, pady=5, sticky='ew')
        self.submit_generate_lawnmower_button.configure(state=DISABLED)
      
        # rotation direction choice
        for (text, value) in rotate_direction_options.items():
        
            rotation_direction_option_button4 = tk.Radiobutton(self.arc_pattern_frame , text = text, variable =  rotate_directions_button_text4, 
                        value = value, indicatoron=0, padx=10,pady=5)
            rotation_direction_option_button4.grid(row = row_index, column = value-1, padx=5, pady=5, sticky="ew")

        row_index +=1
        
        # specifying parameters
        arc_radius_label = Label(self.arc_pattern_frame, text="Radius (m)")
        arc_radius_label.grid(row= row_index, column=0, padx=5, pady=5)
        arc_radius = StringVar()
        arc_radius_entry = Entry(self.arc_pattern_frame,textvariable = arc_radius, width=10).grid(row= row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        arc_angle_label = Label(self.arc_pattern_frame, text="Angle (deg)")
        arc_angle_label.grid(row= row_index, column=0, padx=5, pady=5)
        arc_angle = StringVar()
        arc_angle_entry = Entry(self.arc_pattern_frame,textvariable = arc_angle, width=10).grid(row= row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
        
        visualize_generate_arc_path_button = Button(self.arc_pattern_frame, text=f"Visualize", command=lambda: controller.visualize_arc(relative_option_button_text.get(), arc_radius.get(), arc_angle.get(), rotate_directions_button_text4.get(),orientation_options["Smooth Transition"]))
        visualize_generate_arc_path_button.grid(row = row_index, column=0,columnspan=1, padx=5, pady=5, sticky='ew')   


        self.submit_generate_arc_path_button = Button(self.arc_pattern_frame, text=f"Submit", command=lambda: controller.generate_arc(relative_option_button_text.get(), arc_radius.get(), arc_angle.get(), rotate_directions_button_text4.get(),orientation_options["Smooth Transition"], alg_choice2.get()))
        self.submit_generate_arc_path_button.grid(row = row_index, column=1,columnspan=1, padx=5, pady=5, sticky='ew')   
        self.submit_generate_arc_path_button.configure(state=DISABLED)




    def close(self,controller):
        controller.open_preset_patterns_frame = False
        controller.open_close_preset_patterns_window()


if __name__ == '__main__':

    rospy.init_node('waypoint_gui')
    interface = windows()
    try:
        while not rospy.is_shutdown():
            
            interface.mainloop()
    except Exception as error:
        rospy.logerr("An error occurred:", error)
        
        
        
        
