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
from std_msgs.msg import Bool,Int8,Float32MultiArray, String, Int8MultiArray
from nav_msgs.msg import Path

class windows(Tk):
    def __init__(self, *args, **kwargs): 
        Tk.__init__(self, *args, **kwargs)
        self.pub1 = rospy.Publisher('new_pose_visualization', PoseStamped, queue_size=10)
        self.pub3 = rospy.Publisher("waypoint_plot_visualization", PoseArray, queue_size=10)
        self.pub4 = rospy.Publisher("target_waypoints_list", PoseArray, queue_size=10) # hot used yet
        self.pub10 = rospy.Publisher("motion_controller_state",String, queue_size=1)
        self.pub5 = rospy.Publisher("desired_robot_path", Path, queue_size=10)
        self.pub7 = rospy.Publisher("hold_pose", Bool, queue_size=1)
        self.pub9 = rospy.Publisher("controller_gains", Float32MultiArray, queue_size=10)
        self.pub10 = rospy.Publisher("motion_controller_state",String, queue_size=1)
        self.pub12 = rospy.Publisher("add_waypoint", PoseStamped, queue_size=10)
        self.pub13 = rospy.Publisher("reset_waypoints", Bool, queue_size=10)
        self.pub14 = rospy.Publisher("path_generation_type", Int8MultiArray, queue_size=10)
        self.pub15 = rospy.Publisher("path_orientation_style", Int8MultiArray, queue_size=10)
        
        
        # Setting window icon
        self.icon = PhotoImage(file="/home/andrew/bluerov_waypoint_follower/src/blue_rov_waypoint_interface/scripts/desktop_image_2.png")
        self.iconphoto(True, self.icon)

        #  Current mode of the controller
        self.current_mode = "Disabled"

        # Updates the text in the activate/deactivate button
        self.controller_active = False
        self.open_controller_params_frame = False
        self.open_preset_patterns_frame = False
        self.hold_pose = True
        self.controller_params_window = None
        self.preset_patterns_window = None

        self.target_depth = None


        # Initialize variables 
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

        self.gains = Float32MultiArray()
        self.gains.data =[]   
        self.path_generation_data = Int8MultiArray()
        self.path_generation_data.data = []


        self.max_linear_velocity = 0
        self.min_pwm = 0
        self.max_pwm = 0
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vyaw = 0

         # this stores the last waypoint visualized relative to the current state at the time of visualizing
        self.last_waypoint_rel_to_current_state_visualized = Pose()

        self.last_waypoint_rel_to_current_state_visualized = None
        self.last_waypoint_rel_to_current_state_visualized_transform =np.eye(4)


        self.desired_path = Path()
        self.desired_path.header.frame_id='NED'

        self.goal_waypoints = PoseArray()
        self.goal_waypoints.header.frame_id = 'NED'
        
        self.last_waypoint_transform = np.eye(4)
        self.last_waypoint = Pose()
    

        self.current_pose = PoseWithCovarianceStamped()
        self.init_pose = PoseStamped()
        self.init_pose.header.frame_id = self.current_pose.header.frame_id
        self.init_pose.pose= self.current_pose.pose.pose


        # orientation style along path
        self.orientation_style = Int8MultiArray()
        self.orientation_style.data = []


    
        # Adding a title to the window
        self.wm_title("BlueROV2 Heavy Configuration Interface")

        # creating a frame and assigning it to container
        self.main_container = Frame(self)
        self.main_container.grid(row=0,column=0,padx=10,pady=10, sticky="nsew")

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
        self.target_depth_label.configure(text=f"Target Depth: Not Set")
        self.target_depth_label.grid(row=3, column=0,columnspan=3, padx=20, pady=5,sticky="w")


        # depth setpoint frame
        self.depth_setpoint_frame = LabelFrame(self.depth_frame, text="Set Operating Depth")
        self.depth_setpoint_frame.grid(row = 4, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        # row_index +=1

        # set_target_depth_label = Label(self.depth_frame, text="Set Target Depth (m)")
        # set_target_depth_label.grid(row=0, column=0, columnspan=1, padx=20, pady=5, sticky="w")
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

        # We will now create a dictionary of frames
        self.frames = {}

        # we'll create the frames themselves later but let's add the components to the dictionary.
        # for F in (InfoPage,WaypointFrame,ControlOptionsPage,JoystickFrame,VelocityFrame,PWMFrame,ControllerParamFrame):
        for F in (InfoPage,WaypointFrame,ControlOptionsPage,JoystickFrame,VelocityFrame,PWMFrame):
            frame = F(self.main_container, self)

            # the windows class acts as the root window for the frames.
            # if not F == ControllerParamFrame:
            self.frames[F] = frame

            # if not F == ControllerParamFrame:
            #     frame.grid(row=3, column=0,padx=10,pady=10, sticky="nsew")



        # need to reset after creating teh other frames
        #  Current mode of the controller
        self.current_mode = "Disabled"

        # Updates the text in the activate/deactivate button
        self.controller_active = False

        self.current_mode_label.configure(text=f"Current Mode: {self.current_mode}")
        self.on_off_button.configure(text="Activate Controller")
        
        self.show_frame(InfoPage)

        self.sub1 = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)
        self.sub2 = rospy.Subscriber('/holdpoe', PoseWithCovarianceStamped, self.position_callback)

    def open_close_preset_patterns_window(self):
        if not self.open_preset_patterns_frame:

            try:
                self.preset_patterns_window.destroy()
                self.frames[WaypointFrame].open_preset_patterns_button.configure(text="Open Preset Patterns")
                
            except:
                rospy.logwarn("nothing to close")

        elif self.open_preset_patterns_frame:
            if self.preset_patterns_window:# if it exists try bringing it up 
                try:
                    self.preset_patterns_window.deiconify()
                except:
                    self.preset_patterns_window = PresetPatternsFrame(self.main_container,self)
            else:
                self.preset_patterns_window = PresetPatternsFrame(self.main_container,self)

    def open_close_controller_params_window(self):
        if not self.open_controller_params_frame:

            try:
                self.controller_params_window.destroy()
                self.frames[WaypointFrame].open_controller_params_button.configure(text="Edit Controller Parameters")
                self.frames[VelocityFrame].open_controller_params_button.configure(text="Edit Controller Parameters")

        # controller.open_close_controller_params_window()
            except:
                rospy.logwarn("nothing to close")
        elif self.open_controller_params_frame:
            if self.controller_params_window:# if it exists try bringing it up 
                try:
                    self.controller_params_window.deiconify()
                except:
                    self.controller_params_window = ControllerParamFrame(self.main_container,self)
            else:
                self.controller_params_window = ControllerParamFrame(self.main_container,self)
                
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
        self.current_pose = msg
        self.current_pose_transform = self.get_current_pose_transform()
        self.current_z_label.configure(text=f"Current Depth: {round(self.current_pose.pose.pose.position.z,2)} m")

        # if self.current_mode =="Disabled" | (self.current_mode =="Initialized" and self.target_depth == None):
        #     self.target_depth_label.configure(text=f"Target Depth: {round(self.current_pose.pose.pose.position.z,2)} m")
        
    def get_current_pose_transform(self):
        # convert current pose to transformation matrix relative in NED frame
        trans_matrix = translation_matrix([self.current_pose.pose.pose.position.x,self.current_pose.pose.pose.position.y,self.current_pose.pose.pose.position.z])
        roll, pitch, yaw = euler_from_quaternion([self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w])
        rot_matrix = euler_matrix(roll,pitch,yaw)
        transform  = concatenate_matrices(trans_matrix,rot_matrix)
        return transform
        
    def on_off_button_function(self):
        
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
        # try:
        # indicator tells the subscriber what information is being passed 
        indicator = 5

        # # setting velocity to zero
        self.vx_setpoint = 0
        self.vy_setpoint = 0
        self.vz_setpoint = 0
        self.vyaw_setpoint = 0
        
        # preparing information to be passed
        self.gains.data = [
                    indicator,
                    0,
                    0,
                    0,
                    0
                ]
        
            
        self.pub9.publish(self.gains)
        rospy.loginfo("Setting velocity to zero")
    
    def submit_velocity_setpoint(self,vx,vy,vz,vyaw):
        # try:
        # indicator tells the subscriber what information is being passed 
        indicator = 5

        # # setting velocity to zero
        self.vx_setpoint = round(self.get_float(vx),3)
        self.vy_setpoint = round(self.get_float(vy),3)
        self.vz_setpoint = round(self.get_float(vz),3)
        self.vyaw_setpoint = round(self.get_float(vyaw),3)
        
        # preparing information to be passed
        self.gains.data = [
                    indicator,
                    self.vx_setpoint,        
                    self.vy_setpoint,
                    self.vz_setpoint,
                    self.vyaw_setpoint
                ]
        
            
        self.pub9.publish(self.gains)
        rospy.loginfo("Setting new velocity setpoints")
    
    def submit_target_depth(self,val):

    
        # indicator tells the subscriber what information is being passed
        indicator = 6
        self.target_depth = self.get_float(val)
        # preparing information to be passed
        self.gains.data = [
                indicator,
                self.target_depth
            ]
        
        # updating gui
        # self.target_depth_label.configure(text=f"Current Depth: {round(self.current_pose.pose.pose.position.z,2)} m")
        self.target_depth_label.configure(text=f"Target Depth: {round(self.target_depth,2)} m")

        # publishing information
        self.pub9.publish(self.gains)
        rospy.loginfo(f"Setting target depth to {val} m")
    
    def set_zero_pwm(self):
        indicator = 7

        # setting PWM to zero
        self.x_pwm = 0
        self.y_pwm = 0
        self.z_pwm = 500 
        self.yaw_pwm  = 0
    
        # preparing information to be passed
        self.gains.data = [
                indicator,
                0,
                0,
                500,
                0
            ]
            
        self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
        self.pub9.publish(self.gains)
        rospy.loginfo("Setting pwm to zero")
    
    def submit_pwm_setpoint(self,x_pwm,y_pwm,z_pwm,yaw_pwm):
            
        # indicator tells the subscriber what information is being passed 
        indicator = 7

        # setting PWM to zero
        self.x_pwm = self.get_float(x_pwm)
        self.y_pwm = self.get_float(y_pwm)
        self.z_pwm = self.get_float(z_pwm)
        self.yaw_pwm  = self.get_float(yaw_pwm)
    
        # quick work around for the differing scale and for when z is left blank
        if self.z_pwm == 0.0:
            self.z_pwm  = 500

        if self.z_pwm == 500:
            self.target_depth_label.configure(text=f"Target Depth: Holding current depth")
        else:
            self.target_depth_label.configure(text=f"Target Depth: Manually adjusting depth")

    
        
        # preparing information to be passed
        self.gains.data = [
                indicator,
                self.x_pwm,
                self.y_pwm,
                self.z_pwm,
                self.yaw_pwm
            ]

    
        self.pub9.publish(self.gains)
        rospy.loginfo("Sending PWM setpoints")

    def visualize_waypoint_1(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to current pose"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.get_float(x)
            y = self.get_float(y)
            z = self.get_float(z)
            yaw = self.get_float(yaw)*np.pi / 180

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
            
            
            

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")


            # messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")

    def visualize_waypoint_2(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to last waypoint added"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.get_float(x)
            y = self.get_float(y)
            z = self.get_float(z)
            yaw = self.get_float(yaw)*np.pi / 180

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
            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")

    def visualize_waypoint_3(self,x,y,z,yaw):
      
        """Visualize the single waypoint relative to NED"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.get_float(x)
            y = self.get_float(y)
            z = self.get_float(z)
            yaw = self.get_float(yaw)*np.pi / 180

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
            
            
        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")

    def submit_waypoint_1(self,x,y,z,yaw,path_orientation_style = 1):
        """submit the single waypoint relative to current pose"""
        try:
            

            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.get_float(x)
            y = self.get_float(y)
            z = self.get_float(z)
            yaw = self.get_float(yaw)*np.pi / 180


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
            path_orientation_style = int(self.get_float(path_orientation_style))
            if path_orientation_style ==0:
                rospy.logwarn("No orientation style specified aliong the path, defaulting to RTR")
                path_orientation_style = 4

            self.orientation_style.data.append(path_orientation_style)
            self.pub15.publish(self.orientation_style)

            

            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
            self.pub5.publish(self.desired_path) # shows straight line path 

           



        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")
        
    def submit_waypoint_2(self,x,y,z,yaw,path_orientation_style = 1):
        """submit the single waypoint relative to last waypoint"""
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.get_float(x)
            y = self.get_float(y)
            z = self.get_float(z)
            yaw = self.get_float(yaw)*np.pi / 180


            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            self.desired_path.header.frame_id = 'NED'

            # convert relative transdorm to global transform
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

            # adding to path
            if not self.desired_path.poses:
                start_pose = PoseStamped()
                start_pose.header.frame_id = self.current_pose.header.frame_id
                start_pose.pose= self.current_pose.pose.pose
                self.desired_path.poses.append(start_pose)
    
            self.desired_path.poses.append(pose_stamped_msg)


            #  adding the orientation style along the path 
            path_orientation_style = int(self.get_float(path_orientation_style))
            if path_orientation_style ==0:
                rospy.logwarn("No orientation style specified aliong the path, defaulting to RTR")
                path_orientation_style = 4

            self.orientation_style.data.append(path_orientation_style)
            self.pub15.publish(self.orientation_style)

        
            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
            self.pub5.publish(self.desired_path) # shows straight line path 

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint: {ve}")
        
    def submit_waypoint_3(self,x,y,z,yaw,path_orientation_style = 1):
        """submit the single waypoint relative to NED """
        try:
            roll = 0
            pitch = 0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.get_float(x)
            y = self.get_float(y)
            z = self.get_float(z)
            yaw = self.get_float(yaw)*np.pi / 180
          

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
            path_orientation_style = int(self.get_float(path_orientation_style))
            if path_orientation_style ==0:
                rospy.logwarn("No orientation style specified aliong the path, defaulting to RTR")
                path_orientation_style = 4

            self.orientation_style.data.append(path_orientation_style)
            self.pub15.publish(self.orientation_style)

        
            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints) # shows all target waypoint posed
            self.pub5.publish(self.desired_path) # shows straight line path 
            
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
   
    def submit_controller_gains(self, indicator,kpx, kdx, kix,
             kpy, kdy, kiy,
             kpz, kdz, kiz,
             kpyaw, kdyaw, kiyaw):
    
        try:
            if indicator ==1:
                rospy.loginfo("Sending controller gains for position controller")
            elif indicator ==2:
                rospy.loginfo("Sending controller gains for velcoity controller")
                            
            self.gains.data = [
                    indicator,
                    self.get_float(kpx),
                    self.get_float(kdx),
                    self.get_float(kix),
                    self.get_float(kpy),
                    self.get_float(kdy),
                    self.get_float(kiy),
                    self.get_float(kpz),
                    self.get_float(kdz),
                    self.get_float(kiz),
                    self.get_float(kpyaw),
                    self.get_float(kdyaw),
                    self.get_float(kiyaw),
                ]
            self.pub9.publish(self.gains)

        except ValueError as ve:
            rospy.logerr(f"Invalid input for gains: {ve}")
    
    def submit_max_pwm(self, pwm):
        try:
            indicator = 4
            rospy.loginfo("Setting max pwm signal")
            max_pwm  = float(pwm)
            self.gains.data = [
                    indicator,
                    max_pwm
                    ]
            self.pub9.publish(self.gains)
        except: 
            rospy.logerr("Encountered an error in submit_max_pwm")
        
    def submit_max_linear_velocity(self,velocity):
        try:
            indicator = 3
            rospy.loginfo("Seting max linear velocity")
            # max_linear_velocity  = float(velocity)
            self.gains.data = [
                    indicator,
                    float(velocity)
                    ]
            self.pub9.publish(self.gains)
        except:
            rospy.logerr("Encountered an error in submit_max_linear_velocity")
            
    def submit_max_angular_velocity(self,velocity):
        try:
            indicator = 9
            rospy.loginfo("Seting max angular velocity")
            # self.max_angular_velocity  = float(velocity)
            self.gains.data = [
                    indicator,
                    float(velocity)
                    ]
            self.pub9.publish(self.gains)
        except:
            rospy.logerr("Encountered an error in submit_max_angular_velocity")
     
    def get_float(self,var):
        try:
            var = float(var)
        except:
            var = 0.0
        return var 

    def visualize_path(self, path_indicator, orientation_indicator):
        # rospy.loginfo(int(path_indicator))
        self.path_generation_data.data = [int(path_indicator), int(orientation_indicator)]
        self.pub14.publish(self.path_generation_data)
        rospy.loginfo("Requesting path visualization")
            
    def generate_square(self, relative_indicator, size, rotate_direction, path_orientation_style):
        size = self.get_float(size)
        if size == 0: 
            rospy.logwarn("Cannot create square of size 0, defaulting to 1 m ")
            size = 1 

        if relative_indicator == "1":
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0, path_orientation_style)
        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0, path_orientation_style)

        if rotate_direction == "1":
            rotate = 90
        elif rotate_direction == "2":
            rotate = -90
        elif relative_indicator =="":
            rospy.logwarn("rotation direction not selected, defaulting to cw")
            rotate = 90 
        else: 
            rotate = 90
    

        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style)
        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style)
        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style)
        self.submit_waypoint_2(size,0,0,rotate, path_orientation_style)
    
    def generate_orbit(self, relative_indicator, radius, rotation_angle, rotation_direction,path_orientation_style):
        radius = self.get_float(radius)
        rotation_angle= self.get_float(rotation_angle)*np.pi/180

        if radius == 0: 
            rospy.logwarn("Cannot create circle of radius 0, defaulting to 1 m ")
            radius = 1 

        if relative_indicator == "1":
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0,path_orientation_style)

        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0,path_orientation_style)

        elif relative_indicator =="":
            rospy.logwarn("start point not selected defaulting to current pose")
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        
        if rotation_direction =="1":
            direction_sign = 1

        elif rotation_direction == "2":
            direction_sign = -1
            
        elif rotation_direction == "":
               rospy.logwarn("Rotation direction was not specified, defaulting to cw")
               direction_sign = 1



        # determine how many points you want on the circle
        interval = 20*np.pi/180
        
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
        while i < num_points:    

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

            self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style)
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

        self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style)     
    
    
    def generate_arc(self, relative_indicator, radius, rotation_angle, rotation_direction,path_orientation_style):
        radius = self.get_float(radius)
        rotation_angle= self.get_float(rotation_angle)*np.pi/180

        if radius == 0: 
            rospy.logwarn("Cannot create circle of radius 0, defaulting to 1 m ")
            radius = 1 

        if relative_indicator == "1":
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0, path_orientation_style)

        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0, path_orientation_style)

        elif relative_indicator =="":
            rospy.logwarn("start point not selected defaulting to current pose")
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        

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
        i = 1 
        while i < num_points:    

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
    
            self.submit_waypoint_3(new_x, new_y, z, yaw, path_orientation_style)
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
        yaw = (angle+np.pi/2)*180/np.pi

        # self.submit_waypoint_3(new_x, new_y, z, yaw)     
    
    
    def generate_lawnmower(self, relative_indicator, leg_length, leg_spacing, n_legs, rotate_direction, path_orientation_style):
        leg_length = self.get_float(leg_length)
        leg_spacing = self.get_float(leg_spacing)
        n_legs = self.get_float(n_legs)    

        if leg_length == 0: 
            rospy.logwarn("Cannot create square of size 0, defaulting to 1 m ")
            size = 1 

        if relative_indicator == "1":
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0, path_orientation_style)
        elif relative_indicator =="2":
            self.submit_waypoint_2(0,0,0,0, path_orientation_style)
        elif relative_indicator =="":
            rospy.logwarn("start point not selected, defaulting to current pose")
            self.visualize_waypoint_1(0,0,0,0)
            self.submit_waypoint_1(0,0,0,0, path_orientation_style)

        if rotate_direction == "1":
            rotate = 90
        elif rotate_direction == "2":
            rotate = -90
        elif rotate_direction =="":
            rospy.logwarn("rotation direction not selected, defaulting to cw")
            rotate = 90 

        i = 1

        self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style)
        while i < n_legs:
            self.submit_waypoint_2(leg_spacing,0,0,rotate, path_orientation_style)
            if i == n_legs-1:
                rotate = 0
            else:
                rotate *=-1
            self.submit_waypoint_2(leg_length,0,0,rotate, path_orientation_style)
            i +=1

        # self.submit_waypoint_2(size,0,0,rotate)
        # self.submit_waypoint_2(size,0,0,rotate)
   
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
        # self.path_type_indicator = 1
        # self.orientation_type_indicator = 1

        # get orientation to use along path 

        orientation_options = {"Translate Rotate" : 1,
                                "Rotate Translate" : 2,
                                "Smooth Transition" : 3,
                                "RTR" : 4
                            }
        
        orientation_options_choice = StringVar()


        row_index =0
        
        self.close_waypoint_mode_button = Button(self, text="Disable Waypoint Mode", command=lambda: controller.show_frame(ControlOptionsPage))
        self.close_waypoint_mode_button.grid(row=row_index, column= 0, rowspan=1, columnspan =1,padx=20, pady=5,sticky="ew")

        # Orientation Frame
        self.orientation_options_frame = LabelFrame(self, text="Orientation Along Path")
        self.orientation_options_frame.grid(row = row_index, column=1, rowspan=6, columnspan=1, padx=20, pady=10, sticky="nw")
        row_index +=1
       

        # # Path Generation Frame
        # self.path_generation_frame = LabelFrame(self, text="Path Generation")
        # self.path_generation_frame.grid(row = row_index, column=1, rowspan=6, columnspan=3, padx=20, pady=10, sticky="new")
        # row_index +=1
       
        self.hold_pose_button = Button(self, text="Follow Path", command=lambda: self.hold_pose_button_function(controller))
        # var_text = StringVar()
        if controller.hold_pose:
            self.hold_pose_button.configure(text = "Follow Path")
        else:
            self.hold_pose_button.configure(text = "Hold Pose")
        
        self.hold_pose_button.grid(row= row_index, column=0, rowspan=1, columnspan=1, sticky="ew", padx=20, pady=5)
        row_index+=1
        # self.hold_pose_button = Button(self, text="Follow Path", command=lambda: self.hold_pose_button_function(controller))
        # self.hold_pose_button.grid(row= row_index, column=0, columnspan=1, sticky="ew", padx=20, pady=5)
      
        # self.open_controller_params_button = Button(self, text=f"Edit Controller Parameters", command=lambda: controller.open_controller_params_button_function(controller))
        # self.open_controller_params_button.grid(row= row_index, column=1, columnspan=1, sticky="ew", padx=20, pady=2)
    
        # self.other_button = Button(self, text="Follow Path", command=lambda: controller.hold_pose_button_function())
        # self.other_button.grid(row= row_index, column=2, columnspan=1, sticky="ew", padx=20, pady=2)
        
        
    
         
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
        self.add_waypoints_frame.grid(row = row_index, column=0, columnspan=3, padx=20, pady=10, sticky="ew")
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
       
        '''
        # add waypoints frame 4
        self.add_waypoints_frame_4 = LabelFrame(self.add_waypoints_frame, text="Add a GPS waypoint")
        self.add_waypoints_frame_4.grid(row = row_index, column=1, columnspan=1, padx=20, pady=10, sticky="ew")
        '''

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
    
        visualize_waypoint_button_1 = Button(self.add_waypoints_frame_1, text=f"Visualize", command=lambda: controller.visualize_waypoint_1(x1.get(),y1.get(),z1.get(),yaw1.get()))
        visualize_waypoint_button_1 .grid(row = row_index, column=0,padx=5, pady=5)  
        submit_waypoint_button_1 = Button(self.add_waypoints_frame_1, text=f"Submit", command=lambda: controller.submit_waypoint_1(x1.get(),y1.get(),z1.get(),yaw1.get(), orientation_options_choice.get()))
        submit_waypoint_button_1 .grid(row = row_index, column=1,padx=5, pady=5)   



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
        submit_waypoint_button_2 = Button(self.add_waypoints_frame_2, text=f"Submit", command=lambda: controller.submit_waypoint_2(x2.get(),y2.get(),z2.get(),yaw2.get(),orientation_options_choice.get()))
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

        submit_waypoint_button_3 = Button(self.add_waypoints_frame_3, text=f"Submit", command=lambda: controller.submit_waypoint_3(x3.get(),y3.get(),z3.get(),yaw3.get(), orientation_options_choice.get()))
        submit_waypoint_button_3 .grid(row = row_index, column=1,padx=5, pady=5)   




        # get orientation to use along path 

        for (text, value) in orientation_options.items():
        
            orientation_options_button= tk.Radiobutton(self.orientation_options_frame, text = text, variable =  orientation_options_choice, 
                        value = value, indicatoron=0, padx=10,pady=5)
            # orientation_options_button.grid(row = value, column = 0, padx=5, pady=5,columnspan =2,sticky='ew')
            orientation_options_button.grid(row = value-1, column = 0, padx=10, pady=5, columnspan =1, sticky='ew')

        


        '''
        row_index = 0
        # waypoint relative to GPS
        x_label_4 = Label(self.add_waypoints_frame_4, text="X:")
        x_label_4.grid(row=row_index, column=0, padx=5, pady=5)
        x4 = StringVar()
        x4_entry  = Entry(self.add_waypoints_frame_4,textvariable = x4, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        y_label_4 = Label(self.add_waypoints_frame_4, text="Y:")
        y_label_4.grid(row=row_index, column=0, padx=5, pady=5)
        y4 = StringVar()
        y4_entry = Entry(self.add_waypoints_frame_4,textvariable = y4, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        z_label_4 = Label(self.add_waypoints_frame_4, text="Z:")
        z_label_4.grid(row=row_index, column=0, padx=5, pady=5)
        z4 = StringVar()
        z4_entry = Entry(self.add_waypoints_frame_4,textvariable = z4, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1

        yaw_label_4 = Label(self.add_waypoints_frame_4, text="yaw (deg):")
        yaw_label_4.grid(row=row_index, column=0, padx=5, pady=5)
        yaw4 = StringVar()
        yaw4_entry = Entry(self.add_waypoints_frame_4,textvariable = yaw4, width=10).grid(row=row_index, column=1,padx=5, pady=5, sticky="ew")
        row_index +=1
    
        visualize_waypoint_button_4 = Button(self.add_waypoints_frame_4, text=f"Visualize", command=lambda: controller.visualize_waypoint_4(x4.get(),y4.get(),z4.get(),yaw4.get()))
        visualize_waypoint_button_4 .grid(row = row_index, column=0,padx=5, pady=5)  
        submit_waypoint_button_4 = Button(self.add_waypoints_frame_4, text=f"Submit", command=lambda: controller.submit_waypoint_4(x4.get(),y4.get(),z4.get(),yaw4.get()))
        submit_waypoint_button_4 .grid(row = row_index, column=1,padx=5, pady=5)   

        '''
    
        '''
        # path generation frame
        path_options = {"Straight Line" : 1,
                "Sine Wave" :  2,
                "Arc" :  3}
        
        orientation_options = {"Static" : 1,
                "Facing Next" :  2,
                "Orbit" :  3
                }
        
        relative_options = {"current pose" : 1,
                        "last waypoint" :  2,
                        }
        
        
        path_option_button_text = StringVar()
        orientation_option_button_text = StringVar()
        relative_option_button_text = StringVar()
    
        # create radio buttons for path generation option
        for (text, value) in path_options.items():
        
            path_type_button = tk.Radiobutton(self.path_generation_frame, text = text, variable = path_option_button_text, 
                        value = value, indicatoron=0, padx=5,pady=5,  command=lambda: self.path_generation_button(path_option_button_text.get()))
        
            # row_index = value+1
            path_type_button.grid(row = value-1, column=0,padx=5, pady=5, sticky="w")

        # create radio buttons for orientation options
        for (text, value) in orientation_options.items():
        
            orientation_type_button = tk.Radiobutton(self.path_generation_frame, text = text, variable = orientation_option_button_text, 
                        value = value, indicatoron=0, padx=5,pady=5, command=lambda: self.orientation_generation_button(orientation_option_button_text.get()))
        
            # row_index = value+1
            orientation_type_button.grid(row = value-1, column=1, padx=5, pady=5, sticky="w")

        # visualize_path_button= Button(self.path_generation_frame, text=f"Visualize Path", command=lambda: self.visualize_path(path_option_button_text.get(), orientation_option_button_text.get()))
        visualize_path_button= Button(self.path_generation_frame, text=f"Visualize Path", command=lambda: controller.visualize_path(path_option_button_text.get(), orientation_option_button_text.get()))
        visualize_path_button.grid(row = 0, column=2,padx=5, pady=5, sticky='w')  

        submit_path_button = Button(self.path_generation_frame, text=f"Submit", command=lambda: controller.visualize_path(path_option_button_text.get(), orientation_option_button_text.get()))
        submit_path_button.grid(row = 1, column=2,padx=5, pady=5, sticky='w')  
        
       '''

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
        
        orientation_options_choice2 = StringVar()


        row_index =0
    
        # Orientation Frame
        self.orientation_options_frame2 = LabelFrame(self.preset_patterns_frame, text="Orientation Along Path")
        self.orientation_options_frame2.grid(row = 0, column=3, rowspan=6, columnspan=1, padx=20, pady=10, sticky="new")
         # get orientation to use along path 

        for (text, value) in orientation_options.items():
        
            orientation_options_button2= tk.Radiobutton(self.orientation_options_frame2, text = text, variable =  orientation_options_choice2, 
                        value = value, indicatoron=0, padx=10,pady=5)
            # orientation_options_button.grid(row = value, column = 0, padx=5, pady=5,columnspan =2,sticky='ew')
            orientation_options_button2.grid(row = value-1, column = 0, padx=10, pady=5, columnspan =1, sticky='ew')

       
        # preset patterns frames
        row_index = 0 
        start_point_label = Label(self.preset_patterns_frame, text="Specify start pose relative to: ")
        start_point_label.grid(row=0, column=0, columnspan=2, padx=20, pady=10, sticky='ew')

        for (text, value) in relative_options.items():
        
            relative_option_button = tk.Radiobutton(self.preset_patterns_frame, text = text, variable =  relative_option_button_text, 
                        value = value, indicatoron=0, padx=10,pady=5)
        
            
            # relative_option_button.grid(row = 0, column=value, padx=5, pady=5, sticky="ew")
            relative_option_button.grid(row = 1, column=value-1, padx=5, pady=5, sticky="ew")


        row_index = 2
        # self.square_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Square Pattern")
        # self.square_pattern_frame.grid(row = row_index, column=0, rowspan=1, columnspan=1, padx=20, pady=10, sticky="new")
        # # row_index +=1
        
        # self.lawnmower_frame = LabelFrame(self.preset_patterns_frame , text="Lawnmower Pattern")
        # self.lawnmower_frame.grid(row = row_index, column=1, rowspan=1, columnspan=1, padx=20, pady=10, sticky="new")
        # row_index +=1

    

        self.circle_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Orbit Pattern")
        self.circle_pattern_frame.grid(row = row_index, column=0, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
        row_index +=1

        # self.lawnmower_frame = LabelFrame(self.preset_patterns_frame , text="Lawnmower Pattern")
        # self.lawnmower_frame.grid(row = row_index, column=0, rowspan=1, columnspan=1, padx=20, pady=10, sticky="new")
        # row_index +=1

        self.arc_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Arc Pattern")
        self.arc_pattern_frame.grid(row = row_index, column=0, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
        row_index +=1


        
        self.lawnmower_frame = LabelFrame(self.preset_patterns_frame , text="Lawnmower Pattern")
        self.lawnmower_frame.grid(row = row_index, column=0, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
        row_index +=1
        

        self.square_pattern_frame = LabelFrame(self.preset_patterns_frame , text="Square Pattern")
        self.square_pattern_frame.grid(row = row_index, column=0, rowspan=1, columnspan=2, padx=20, pady=10, sticky="new")
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

        submit_generate_square_path_button = Button(self.square_pattern_frame, text=f"Submit", command=lambda: controller.generate_square(relative_option_button_text.get(), square_size.get(),rotate_directions_button_text1.get(),orientation_options_choice2.get()))
        submit_generate_square_path_button.grid(row = row_index, column=0, columnspan =2, padx=5, pady=5, sticky = 'ew')   

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
        
        submit_generate_circle_path_button = Button(self.circle_pattern_frame, text=f"Submit", command=lambda: controller.generate_orbit(relative_option_button_text.get(), radius.get(), angle.get(), rotate_directions_button_text2.get(), orientation_options["Smooth Transition"]))
        submit_generate_circle_path_button.grid(row = row_index, column=0,columnspan=2, padx=5, pady=5, sticky='ew')   


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


        submit_generate_lawnmower_button = Button(self.lawnmower_frame, text=f"Submit", command=lambda: controller.generate_lawnmower(relative_option_button_text.get(), leg_length.get(), leg_spacing.get(), n_legs.get(),rotate_directions_button_text3.get(), orientation_options["Translate Rotate"]))
        submit_generate_lawnmower_button.grid(row = row_index, column=0,columnspan=2, padx=5, pady=5, sticky='ew')


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
        
        submit_generate_arc_path_button = Button(self.arc_pattern_frame, text=f"Submit", command=lambda: controller.generate_arc(relative_option_button_text.get(), arc_radius.get(), arc_angle.get(), rotate_directions_button_text4.get(),orientation_options["Smooth Transition"]))
        submit_generate_arc_path_button.grid(row = row_index, column=0,columnspan=2, padx=5, pady=5, sticky='ew')   



    def close(self,controller):
        controller.open_preset_patterns_frame = False
        controller.open_close_preset_patterns_window()


if __name__ == '__main__':

    rospy.init_node('waypoint_gui')
    interface = windows()
    while not rospy.is_shutdown():

        interface.mainloop()

