#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped,PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, euler_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, euler_from_quaternion
# from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox, Frame, Toplevel
from tkinter import *
from tkinter.ttk import *
from std_msgs.msg import Bool,Int8,Float32MultiArray, String
from nav_msgs.msg import Path

#  should consider switching things from pose to poseStamped
class WaypointGui:  
    def __init__(self, master):

        # self.icon = PhotoImage(file="scripts/desktop_image_2.png")
        self.icon = PhotoImage(file="/home/andrew/bluerov_waypoint_follower/src/blue_rov_waypoint_interface/scripts/desktop_image_2.png")
     
        # Set it as the window icon.
        master.iconphoto(True, self.icon)
        
        self.current_mode = "Disabled"

        self.controller_active = False
        self.plots_active = False
       

        # dictionary for geting velocity mode field entries
        # Note:sometimes the names get complicated, for example getting field entry for depth from velocity mode vs other modes so i create a dictionary for each mode and extract the information this way 
        self.velocity_mode_dict = {
            "vx" : "vx_entry",
            "vy" : "vy_entry",
            "vz" : "vz_entry",
            "vyaw" : "vyaw_entry",
            "target_depth" : "depth_entry_vel_mode"
            }
        # dictionary for geting pwm mode field entries
        self.pwm_mode_dict= {
            "x_pwm" : "x_pwm_entry",
            "y_pwm" : "y_pwm_entry",
            "z_pwm" : "z_pwm_entry",
            "yaw_pwm" : "yaw_pwm_entry",
            "target_depth" : "depth_entry_pwm_mode"
            }
        


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
        self.gains.data = [
            0,
            self.kp_x,
            self.kd_x,
            self.ki_x,
            self.kp_y,
            self.kd_y,
            self.ki_y,
            self.kp_z,
            self.kd_z,
            self.ki_z,
            self.kp_yaw,
            self.kd_yaw,
            self.ki_yaw

        ]   
        self.max_linear_velocity = 0
        self.min_pwm = 0
        self.max_pwm = 0
        self.v_testing_toggle ="off"
        self.vx = 0
        self.vy = 0
        self.vz = 0
        self.vyaw = 0


        self.goal_waypoints = PoseArray()
        self.goal_waypoints.header.frame_id = 'NED'
        
        self.last_waypoint_transform = np.eye(4)
        self.last_waypoint = Pose()
    

        self.current_pose = PoseWithCovarianceStamped()
        self.init_pose = PoseStamped()
        self.init_pose.header.frame_id = self.current_pose.header.frame_id
        self.init_pose.pose= self.current_pose.pose.pose

        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = self.current_pose.header.frame_id
        self.home_pose.pose= self.current_pose.pose.pose
       

        
        self.motion_control_state = Bool()
        self.motion_control_state = False

        self.hold_pose = Bool()
        self.hold_pose = False
        # self.current_pose_transform = np.eye(4)
        
        # this stores the last waypoint visualized relative to the current state at the time of visualizing
        self.last_waypoint_rel_to_current_state_visualized = Pose()

        self.last_waypoint_rel_to_current_state_visualized = None
        self.last_waypoint_rel_to_current_state_visualized_transform =np.eye(4)

        
        self.desired_path = Path()
        self.desired_path.header.frame_id='NED'

        self.desired_path.poses.append(self.init_pose)

        self.start_waypoint_index = Int8()
        self.start_waypoint_index = 0
        # initialize the GUI
        self.master = master
        self.master.title("Control Modes")

        # Create the main container
        self.main_frame = Frame(master)
        self.main_frame.pack(padx=5, pady=5, fill="both", expand=True)
        self.create_main_frame()



        # Initialize all frames and then forget them 
        self.create_main_frame()
        self.create_on_off_frame()

        self.create_controller_modes_frame()
        self.forget(self.controller_mode_options)

        self.create_joystick_mode_frame()
        self.forget(self.joystick_mode_frame)

        self.create_velocity_setpoint_mode_frame()
        self.forget(self.velocity_setpoint_mode_frame)
        
        self.create_pwm_frame()
        self.forget(self.pwm_mode_frame)

        self.create_waypoint_mode_frame()
        self.forget(self.waypoint_mode_frame)


   
       


        # # Major Grid 
        # self.open_joystick_mode_button = Button(self.main_frame, text="Activate Joystick Mode", command=self.open_joystick_mode_window)
        # self.open_joystick_mode_button.grid(row=0, column= 3,padx=20, pady=20,sticky="ew")
        
        # self.open_waypoint_mode_button = Button(self.main_frame, text="Activate Waypoint Mode", command=self.open_waypoint_mode_window)
        # self.open_waypoint_mode_button.grid(row=1, column= 3,padx=20, pady=20,sticky="ew")


        # self.open_velocity_mode_button = Button(self.main_frame, text="Activate Velocity Setpoint Mode", command=self.open_velocity_setpoint_mode_window)
        # self.open_velocity_mode_button.grid(row=2, column= 3,padx=20, pady=20,sticky="ew")

        # self.open_pwm_mode_button = Button(self.main_frame, text="Open Manual PWM Mode", command=self.open_manual_PWM_window)
        # self.open_pwm_mode_button.grid(row=3, column = 3,padx=20, pady=20,sticky="ew")

        # self.open_controller_params_button = Button(self.main_frame, text="Open Controller Parameters", command=self.open_controller_params_window)
        # self.open_controller_params_button.grid(row=4, column=3 ,padx=20, pady=20,sticky="ew")


        # Initialize ROS node for publishing
        # publishes the last waypoint pose in rviz
        self.pub1 = rospy.Publisher('new_pose_visualization', PoseStamped, queue_size=10)
        # publishes the controller state on or off
        # self.pub2 = rospy.Publisher("motion_control_state",Bool, queue_size=10)

        self.pub10 = rospy.Publisher("motion_controller_state",String, queue_size=1)

        # plots the list of waypoints
        self.pub3 = rospy.Publisher("waypoint_plot_visualization", PoseArray, queue_size=10)

        # publishes the array to send the motion controller 
        self.pub4 = rospy.Publisher("target_waypoints_list", PoseArray, queue_size=10)
    

        #publishes the expected path to follow
        self.pub5 = rospy.Publisher("desired_robot_path", Path, queue_size=10)

        #  publishes True when waypoint_index is reset
        self.pub6 = rospy.Publisher("wapoint_index_reset", Int8, queue_size=10)

        # publishes True when waypoint_index is reset
        self.pub7 = rospy.Publisher("hold_pose", Bool, queue_size=10)

        # publishes True when waypoint_index is reset
        self.pub8 = rospy.Publisher("hold_pose_waypoint", PoseStamped, queue_size=10)
        # publishes True when waypoint_index is reset
        self.pub9 = rospy.Publisher("controller_gains", Float32MultiArray, queue_size=10)
        
        self.pub11 = rospy.Publisher("toggle_velocity_plot", Bool, queue_size=10)





        # Initialize subscriber
        # Subscribe to the local position topic
        # self.sub1 = rospy.Subscriber('current_state_test', PoseWithCovarianceStamped, self.position_callback)
        # self.sub1 = rospy.Subscriber('/dvl/local_position', PoseWithCovarianceStamped, self.position_callback)
        self.sub1 = rospy.Subscriber('/state', PoseWithCovarianceStamped, self.position_callback)

        # self.disable_motion_control()

    # helper functions
    def create_user_input_field(self, frame, display_text, label_name, row_index, column_index = 0):
        label = Label(frame, text=f"{display_text}")
        label.grid(row=row_index, column=column_index,columnspan=1, padx=5, pady=5, sticky="ew")

        setattr(self, f"{label_name}", Entry(frame, width = 10))
        getattr(self, f"{label_name}").grid(row=row_index, column=column_index+1, padx=5, pady=5)

    def get_user_input(self,label):
        try:
            var =  float(getattr(self,f"{label}").get())

        except:
            var = 0.0
        return var
       
    def forget(self,widget):
        widget.grid_forget()    


    #  main frame
    def create_main_frame(self):
       
        self.create_status_frame()
        self.create_on_off_frame()


    #  current status
    def create_status_frame(self):
        # current status frame
        self.current_status_frame = LabelFrame(self.main_frame, text="Status")
        self.current_status_frame.grid(row = 0, column=0, columnspan=1, pady=10, sticky="ew")

        # Display current mode in current status frame
        self.current_mode_label = Label(self.current_status_frame, text=f"Current mode: {self.current_mode}")
        self.current_mode_label.grid(row=0, column=0, padx=20, pady=20,sticky="w")


    # activate or deactivate the controller
    def create_on_off_frame(self):
        
        self.on_off_frame = LabelFrame(self.main_frame, text="Toggle on/off")
        self.on_off_frame.grid(row = 1, column=0, columnspan=1, pady=10, sticky="ew")

        if self.controller_active:
            # Deactivate button in on off frame
            self.on_off_button = Button(self.on_off_frame, text="Deactivate Controller", command=self.deactivate_controller)
        
        elif not self.controller_active:
            # Activate button in on off frame
            self.on_off_button = Button(self.on_off_frame, text="Activate Controller", command=self.activate_controller)
            
        self.on_off_button.grid(row=0, column= 0,padx=20, pady=20, sticky = "w")

        # if self.plots_active:
        #     # Deactivate button in on off frame
        #     self.plots_button = Button(self.on_off_frame, text="Close Velocity Plot", command=self.close_plots)
        
        # elif not self.controller_active:
        #     # Activate button in on off frame
        #     self.plots_button = Button(self.on_off_frame, text="Show Velocity Plot", command=self.show_plots)
            
        # self.plots_button.grid(row=1, column= 0,padx=20, pady=20, sticky = "w")


     

    def activate_controller(self):
        # Update mode to initialized
        self.current_mode = "Initialized"
        rospy.loginfo(f"Controller is initialized")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  

        # update status
        self.create_status_frame()

        # update on off button 
        self.create_on_off_frame()

        # create and display the frame that holds the controller modes
        self.create_controller_modes_frame()


    def deactivate_controller(self):

        # Update mode to disabled
        self.current_mode = "Disabled"
        rospy.loginfo(f"Controller is disabled")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = False

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.controller_mode_options)
        self.forget(self.joystick_mode_frame)
        self.forget(self.velocity_setpoint_mode_frame)
        self.forget(self.pwm_mode_frame)
        self.forget(self.waypoint_mode_frame)


        # update status
        self.create_status_frame()

        # update on off button 
        self.create_on_off_frame()
      
    def show_plots(self):
    

        rospy.loginfo(f"Opening velocity plots")

        self.plots_active = True
        # Pubslish the new state
        self.pub11.publish(self.plots_active)
        
       

        # Erase the current frames
        # self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  

        # update status
        # self.create_status_frame()

        # update on off button 
        self.create_on_off_frame()

        # create and display the frame that holds the controller modes
        # self.create_controller_modes_frame()


    def close_plots(self):

        rospy.loginfo(f"Closing velocity plots")

        self.plots_active = False
        # Pubslish the new state
        self.pub11.publish(self.plots_active)
        
       

        # Erase the current frames
        # self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  

        # update status
        # self.create_status_frame()

        # update on off button 
        self.create_on_off_frame()

        # create and display the frame that holds the controller modes
        # self.create_controller_modes_frame()

      

    # controller modes selection
    def create_controller_modes_frame(self,start_row=3):

        row_index = 0

        self.controller_mode_options= LabelFrame(self.main_frame, text="Controller modes")
        self.controller_mode_options.grid(row = start_row, column=0, columnspan=1, pady=10, sticky="ew")

        self.open_joystick_mode_button = Button(self.controller_mode_options, text="Activate Joystick Mode", command=self.activate_joystick_mode)
        self.open_joystick_mode_button.grid(row=row_index, column= 0,padx=20, pady=10,sticky="ew")
        row_index+=1
        
        self.open_waypoint_mode_button = Button(self.controller_mode_options, text="Activate Waypoint Mode", command=self.activate_waypoint_mode)
        self.open_waypoint_mode_button.grid(row=row_index, column= 0,padx=20, pady=10,sticky="ew")
        row_index+=1
        
        self.open_velocity_mode_button = Button(self.controller_mode_options, text="Activate Velocity Mode", command=self.activate_velocity_setpoint_mode)
        self.open_velocity_mode_button.grid(row=row_index, column= 0,padx=20, pady=10,sticky="ew")
        row_index+=1

        self.open_pwm_mode_button = Button(self.controller_mode_options, text="Activate PWM Mode", command=self.activate_pwm_mode)
        self.open_pwm_mode_button.grid(row=row_index, column = 0,padx=20, pady=10,sticky="ew")
        row_index+=1

        self.open_controller_params_button = Button(self.controller_mode_options, text="Open Controller Parameters", command=self.create_controller_params_window)
        self.open_controller_params_button.grid(row=row_index, column=0 ,padx=20, pady=10,sticky="ew")
        row_index+=1




    # joystick mode
 
    def activate_joystick_mode(self):

        # Update mode to initialized
        self.current_mode = "Joystick"
        rospy.loginfo("Activating joystick mode")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.controller_mode_options)

        # make frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_joystick_mode_frame()


    def deactivate_joystick_mode(self):
        
        #  Update mode to disabled
        self.current_mode = "Initialized"
        rospy.loginfo(f"The controller is waiting for the mode to be selected")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.joystick_mode_frame)

        # create new frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_controller_modes_frame()
      

    def create_joystick_mode_frame(self):

        # create joystick mode frame
        self.joystick_mode_frame= LabelFrame(self.main_frame, text="Joystick Mode Options")
        self.joystick_mode_frame.grid(row = 2, column=0, columnspan=1, pady=10, sticky="ew")

    
        # deactivate joystick mode button
        self.deactivate_joystick_mode_button = Button(self.joystick_mode_frame, text=f"Deactivate joystick mode", command= self.deactivate_joystick_mode)
        self.deactivate_joystick_mode_button.grid(row= 0, column=0, padx=20, pady=20, sticky = "w" )
    
    
    # velocity setpoint mode 
    def activate_velocity_setpoint_mode(self):

        # Update mode
        self.current_mode = "velocity"
        rospy.loginfo("Activating velocity setpoint mode")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.controller_mode_options)
        

        # make frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_velocity_setpoint_mode_frame()
    
    
    def deactivate_velocity_setpoint_mode(self):
        
        #  Update mode 
        self.current_mode = "Initialized"
        rospy.loginfo(f"The controller is waiting for the mode to be selected")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.velocity_setpoint_mode_frame)

        # create new frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_controller_modes_frame()
      
  
    def create_velocity_setpoint_mode_frame(self):
        
        # create velocity mode frame
        self.velocity_setpoint_mode_frame = LabelFrame(self.main_frame, text="Velocity Mode Options")
        self.velocity_setpoint_mode_frame.grid(row = 3, column=0, columnspan=1, pady=10, sticky="ew")

        row_index = 0 

        # deactivate velocity mode button
        self.deactivate_velocity_setpoint_mode_button = Button(self.velocity_setpoint_mode_frame, text=f"Deactivate velocity mode", command= self.deactivate_velocity_setpoint_mode)
        self.deactivate_velocity_setpoint_mode_button.grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        row_index +=1

        # Edit controller params button
        self.open_controller_params_button = Button(self.velocity_setpoint_mode_frame, text=f"Edit controller gains", command= self.create_controller_params_window)
        self.open_controller_params_button .grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        row_index +=1
        
        # set zero velocity button
        self.set_velocity_to_zero_button = Button(self.velocity_setpoint_mode_frame, text=f"Set velocity to zero", command= self.set_zero_velocity)
        self.set_velocity_to_zero_button.grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        row_index +=1

        # depth setpoint frame
        self.depth_setpoint_velocity_frame = LabelFrame(self.velocity_setpoint_mode_frame, text="Set Operating Depth")
        self.depth_setpoint_velocity_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        row_index +=1

        # velocity setpoint frame
        self.velocity_setpoint_frame= LabelFrame(self.velocity_setpoint_mode_frame, text="Velocity Setpoints")
        self.velocity_setpoint_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
    

        # creating depth label in depth setpoint frame
        row_index = 0


        self.create_user_input_field(frame = self.depth_setpoint_velocity_frame,
                                     display_text="Target depth (m)",
                                     label_name="depth_entry_vel_mode",
                                     row_index=row_index,
                                     )
        row_index +=1

        submit_depth_vel_mode_button = Button(self.depth_setpoint_velocity_frame, text=f"Submit", command=self.submit_target_depth)
        submit_depth_vel_mode_button.grid(row=row_index, column=1,columnspan=1, padx = 20, pady=10, sticky="w")  

       
        # creating velocity setpoint labels in velocity setpoint frame
        row_index = 0
    
        self.create_user_input_field(frame = self.velocity_setpoint_frame,
                                     display_text="x velocity (m/s)",
                                     label_name="vx_entry",
                                     row_index=row_index,
                                     )
        row_index +=1

        self.create_user_input_field(frame = self.velocity_setpoint_frame,
                                     display_text="y velocity (m/s)",
                                     label_name="vy_entry",
                                     row_index=row_index,
                                     )
        row_index +=1

        self.create_user_input_field(frame = self.velocity_setpoint_frame,
                                     display_text="z velocity (m/s)",
                                     label_name="vz_entry",
                                     row_index=row_index,
                                     )
        row_index +=1

        self.create_user_input_field(frame = self.velocity_setpoint_frame,
                                     display_text="yaw velocity (deg/s)",
                                     label_name="vyaw_entry",
                                     row_index=row_index,
                                     )
        row_index +=1

        submit_velocity_button = Button(self.velocity_setpoint_frame, text=f"Submit", command=self.submit_velocity_setpoint)
        submit_velocity_button.grid(row = row_index, column=1,padx=5, pady=5)          
        


    # Manual PWM Mode
    def activate_pwm_mode(self):
        
        # Update mode
        self.current_mode = "manual pwm"
        rospy.loginfo("Activating pmw mode")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.controller_mode_options)
        

        # make frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_pwm_frame()
    
    
    def deactivate_pwm_mode(self):
        
        #  Update mode 
        self.current_mode = "Initialized"
        rospy.loginfo(f"The controller is waiting for the mode to be selected")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.pwm_mode_frame)

        # create new frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_controller_modes_frame()
      

    def create_pwm_frame(self):
       

        # create pwm mode frame
        self.pwm_mode_frame = LabelFrame(self.main_frame, text="PWM Mode Options")
        self.pwm_mode_frame.grid(row = 3, column=0, columnspan=1, pady=10, sticky="ew")

        row_index = 0 

        # deactivate PWM mode button
        self.deactivate_pwm_mode_button = Button(self.pwm_mode_frame, text=f"Deactivate PWM mode", command= self.deactivate_pwm_mode)
        self.deactivate_pwm_mode_button.grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        row_index +=1
        
        # set zero PWM button
        self.set_pwm_to_zero_button = Button(self.pwm_mode_frame, text=f"Set pwm to zero", command= self.set_zero_pwm)
        self.set_pwm_to_zero_button.grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        row_index +=1

        # depth setpoint frame
        self.depth_setpoint_pwm_frame = LabelFrame(self.pwm_mode_frame, text="Set Operating Depth")
        self.depth_setpoint_pwm_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        row_index +=1

        # PWM setpoint frame
        self.pwm_setpoint_frame= LabelFrame(self.pwm_mode_frame, text="PWM Setpoints")
        self.pwm_setpoint_frame.grid(row = row_index, column=0, columnspan=1, padx=20, pady=10, sticky="ew")
        
        # creating depth label in depth setpoint frame
        row_index = 0


        self.create_user_input_field(frame = self.depth_setpoint_pwm_frame,
                                     display_text="Target depth (m)",
                                     label_name="depth_entry_pwm_mode",
                                     row_index=row_index,
                                     )
        row_index +=1
        submit_depth_pwm_mode_button = Button(self.depth_setpoint_pwm_frame, text=f"Submit", command=self.submit_target_depth)
        submit_depth_pwm_mode_button.grid(row= 1, column=1,pady=5)             
        

        # creating pwm label in pwm setpoint frame
        row_index = 0
        self.create_user_input_field(frame = self.pwm_setpoint_frame,
                                     display_text="x pwm (-1000, 1000)",
                                     label_name="x_pwm_entry",
                                     row_index=row_index,
                                     )
        row_index +=1

        self.create_user_input_field(frame = self.pwm_setpoint_frame,
                                     display_text="y pwm (-1000, 1000)",
                                     label_name="y_pwm_entry",
                                     row_index=row_index,
                                     )
        row_index +=1

        # note: set pwm entry from 1 to 1000 so if it is left blank then it is set to 500 instead of 0
        self.create_user_input_field(frame = self.pwm_setpoint_frame,
                                     display_text="z pwm (1, 1000)",
                                     label_name="z_pwm_entry",
                                     row_index=row_index,
                                     )
        row_index +=1
        
        self.create_user_input_field(frame = self.pwm_setpoint_frame,
                                     display_text="yaw pwm (-1000, 1000)",
                                     label_name="yaw_pwm_entry",
                                     row_index=row_index,
                                     )
        row_index +=1
        
        self.submit_pwm_button = Button(self.pwm_setpoint_frame, text=f"Submit", command=self.submit_pwm_setpoint)
        self.submit_pwm_button.grid(row = row_index, column=1,pady=5)    




    # Waypoint modde
    def activate_waypoint_mode(self):
        
        # Update mode
        self.current_mode = "waypoint"
        
        rospy.loginfo("Activating waypoint follower mode")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.controller_mode_options)
        

        # make frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_waypoint_mode_frame()

           

        if self.hold_pose:
            self.hold_pose = False
            rospy.loginfo("Continuing to waypoints")
            self.pub7.publish(self.hold_pose)
        
        elif not self.hold_pose:
            self.hold_pose = True
            rospy.loginfo("Requested position hold")
            self.pub7.publish(self.hold_pose)

            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            pose_stamped_msg.pose = self.current_pose.pose.pose
    
            self.pub8.publish(pose_stamped_msg)
    

    def deactivate_waypoint_mode(self):
        
        #  Update mode 
        self.current_mode = "Initialized"
        rospy.loginfo(f"The controller is waiting for the mode to be selected")

        # Pubslish the new state
        self.pub10.publish(self.current_mode)
        
        # Toggle controller active variable
        self.controller_active = True

        # Erase the current frames
        self.forget(self.current_status_frame)
        self.forget(self.on_off_frame)  
        self.forget(self.waypoint_mode_frame)

        # create new frames
        self.create_status_frame()
        self.create_on_off_frame()
        self.create_controller_modes_frame()
      
    
    def create_waypoint_mode_frame(self):

        # create waypoint mode frame
        self.waypoint_mode_frame = LabelFrame(self.main_frame, text="Waypoint Mode Options")
        self.waypoint_mode_frame.grid(row = 3, column=0, columnspan=1, pady=10, sticky="ew")

        row_index = 0 

        # deactivate PWM mode button
        self.deactivate_waypoint_mode_button = Button(self.waypoint_mode_frame, text=f"Deactivate waypoint mode", command= self.deactivate_waypoint_mode)
        self.deactivate_waypoint_mode_button.grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        row_index +=1

        # Edit controller params button
        self.open_controller_params_button = Button(self.waypoint_mode_frame, text=f"Edit controller gains", command= self.create_controller_params_window)
        self.open_controller_params_button .grid(row= row_index, column=0, padx=20, pady=10, sticky = "w" )
        row_index +=1
        
        # Add waypoints frame
        self.waypoints_frame = LabelFrame(self.waypoint_mode_frame, text="Add Waypoints")
        self.waypoints_frame.grid(row = row_index, column=0, columnspan=3, pady=10, sticky="ew")
        row_index +=1

        # motion controller frame
        self.motion_controller_frame = LabelFrame(self.waypoint_mode_frame, text = "Motion Controller")
        self.motion_controller_frame.grid(row=row_index, column=0, pady=10, sticky="ew")
        row_index +=1
      
        # generate waypoints frame
        self.generate_waypoints_frame = LabelFrame(self.waypoint_mode_frame, text = "Generate test patterns")
        self.generate_waypoints_frame.grid(row=row_index, column=0, pady=10, sticky="ew")
        




        self.add_waypoint_frame_1 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nNED frame")
        self.add_waypoint_frame_1.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")
        # add waypoints frame row 1 column 2
        self.add_waypoint_frame_2 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe last waypoint added")
        self.add_waypoint_frame_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        # add waypoints frame row 2 column 1
        self.add_waypoint_frame_3= LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe current pose")
        self.add_waypoint_frame_3.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")


        self.create_waypoint_fields(self.add_waypoint_frame_1, '1')
        self.create_waypoint_fields(self.add_waypoint_frame_2, '2')
        self.create_waypoint_fields(self.add_waypoint_frame_3, '3')

        # Add buttons beside the waypoint frames
        # Main Frame row 2
        row_index = 0
        column_index = 0
        column_span = 2
    

        # Add buttons beside the waypoint frames
        # main frame row 2
        row_index = 0
        column_index = 0
        column_span = 2
    
        # # column 1
        # self.toggle_motion_controller = Button(self.motion_controller_frame, text="Activate controller", command=self.enable_motion_control)
        # self.toggle_motion_controller.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2,  pady=2)
        # row_index +=1

        self.publish_to_controller= Button(self.motion_controller_frame, text="Update waypoints", command=self.publish_goal_waypoints)
        self.publish_to_controller.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1

        self.erase_waypoints_button = Button(self.motion_controller_frame, text="Erase Waypoints", command=self.erase_waypoints)
        self.erase_waypoints_button.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1
        
        self.visualize_goal_waypoints = Button(self.motion_controller_frame, text="Visualize plan", command=self.visualize_waypoints)
        self.visualize_goal_waypoints.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1

        self.pause= Button(self.motion_controller_frame, text="Toggle hold pose", command=self.toggle_hold_pose)
        self.pause.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1

        # main frame row 3 column 1
        row_index = 0
        column_index = 0
        column_span = 2
     
        # generate waypoints frame row 1 column 1
        row_index = 0
        column_index = 0
        column_span = 2

        self.make_square= Button(self.generate_waypoints_frame, text="Generate Square", command=self.generate_square)
        self.make_square.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew",  padx=2, pady=2)
        row_index +=1
            



    #  Controller parameters window
    def create_controller_params_window(self):
       
        try:  
            self.controller_params_window.deiconify()  
            
        except:  
        
            # Create a new window
            self.controller_params_window = Toplevel(self.master)
            self.controller_params_window.title("controller params")
            # Position Controller Parameters
            self.position_controller_params = LabelFrame(self.controller_params_window, text="Position Controller Gains")
            self.position_controller_params.grid(row = 0, column=0, columnspan=1, pady=10, sticky="ew")
            self.create_pid_gains(self.position_controller_params,'1')

            # Velocity Controller Parameters
            self.velocity_controller_params = LabelFrame(self.controller_params_window, text="Velocity Controller Gains")
            self.velocity_controller_params.grid(row = 1, column=0, columnspan=1, pady=10, sticky="ew")
            self.create_pid_gains(self.velocity_controller_params,'2')


            # Saturation Parameters
            self.saturation_params = LabelFrame(self.controller_params_window, text="Saturation Parameters")
            self.saturation_params.grid(row = 2, column=0, columnspan=1, pady=10, sticky="ew")
            self.create_saturation_params(self.saturation_params)

            # self.create_pid_gains(self.PWM_pid_gains,'2')

            # Button to return to the main window
            self.controller_params_window_back_button = Button(self.controller_params_window, text="Close Window", command=self.controller_params_window.destroy)
            self.controller_params_window_back_button.grid(row = 3, column=0, sticky="ew", padx=2, pady=2)


    def create_pid_gains(self, frame, suffix):
        """ Helper function to create labeled entry widgets """
        # row 1 
        kp_label = Label(frame, text="kp")
        kp_label.grid(row=0, column=1, padx=5, pady=5)

        kd_label = Label(frame, text="kd")
        kd_label.grid(row=0, column=2, padx=5, pady=5)       

        ki_label = Label(frame, text="ki")
        ki_label.grid(row=0, column=3, padx=5, pady=5)  



        # row 2
        x_label = Label(frame, text="x")
        x_label.grid(row=1, column=0, padx=5, pady=5)
        setattr(self, f"kp_x_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kp_x_entry_{suffix}").grid(row=1, column=1, padx=5, pady=5)
        setattr(self, f"kd_x_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kd_x_entry_{suffix}").grid(row=1, column=2, padx=5, pady=5)
        setattr(self, f"ki_x_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"ki_x_entry_{suffix}").grid(row=1, column=3, padx=5, pady=5)

        # row 2
        y_label = Label(frame, text="y")
        y_label.grid(row=2, column=0, padx=5, pady=5)
        setattr(self, f"kp_y_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kp_y_entry_{suffix}").grid(row=2, column=1, padx=5, pady=5)
        setattr(self, f"kd_y_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kd_y_entry_{suffix}").grid(row=2, column=2, padx=5, pady=5)
        setattr(self, f"ki_y_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"ki_y_entry_{suffix}").grid(row=2, column=3, padx=5, pady=5)


         # row 3
        z_label = Label(frame, text="z")
        z_label.grid(row=3, column=0, padx=5, pady=5)
        setattr(self, f"kp_z_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kp_z_entry_{suffix}").grid(row=3, column=1, padx=5, pady=5)
        setattr(self, f"kd_z_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kd_z_entry_{suffix}").grid(row=3, column=2, padx=5, pady=5)
        setattr(self, f"ki_z_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"ki_z_entry_{suffix}").grid(row=3, column=3, padx=5, pady=5)

         # row 4
        yaw_label = Label(frame, text="yaw")
        yaw_label.grid(row=4, column=0, padx=5, pady=5)
        setattr(self, f"kp_yaw_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kp_yaw_entry_{suffix}").grid(row=4, column=1, padx=5, pady=5)
        setattr(self, f"kd_yaw_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kd_yaw_entry_{suffix}").grid(row=4, column=2, padx=5, pady=5)
        setattr(self, f"ki_yaw_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"ki_yaw_entry_{suffix}").grid(row=4, column=3, padx=5, pady=5)

        submit_gains_button = Button(frame, text=f"Submit gains", command=lambda: self.submit_controller_gains(suffix))
        submit_gains_button.grid(row=5, column=3,pady=10)


    def create_saturation_params(self, frame):
        """ Helper function to create labeled entry widgets """
        row_index = 0

        self.create_user_input_field(frame = frame,
                                     display_text="max linear velocity (m/s)",
                                     label_name="max_linear_velocity_entry",
                                     row_index=row_index,
                                     )
        self.submit_linear_velocity_button = Button(frame, text=f"Submit", command=self.submit_max_linear_velocity)
        self.submit_linear_velocity_button.grid(row= row_index, column=2,pady=10)
        row_index +=1


        self.create_user_input_field(frame = frame,
                                     display_text="max angular velocity (deg/s)",
                                     label_name="max_angular_velocity_entry",
                                     row_index=row_index,
                                     )
       
        self.submit_angular_velocity_button = Button(frame, text=f"Submit", command=self.submit_max_angular_velocity)
        self.submit_angular_velocity_button.grid(row= row_index, column=2,pady=10)
        row_index +=1



        self.create_user_input_field(frame = frame,
                                     display_text="max x and y pwm (0 - 1000)",
                                     label_name="x_y_pwm_entry",
                                     row_index=row_index,
                                     )
        self.submit_x_y_pwm_button = Button(frame, text=f"Submit", command= self.submit_max_x_y_pwm)
        self.submit_x_y_pwm_button.grid(row= row_index, column=2,pady=10)

        row_index +=1
        





    # publishing functions
    def set_zero_velocity(self):
        try:
            # indicator tells the subscriber what information is being passed 
            indicator = 5

            # setting velocity to zero
            self.vx_setpoint = 0
            self.vy_setpoint = 0
            self.vz_setpoint = 0
            self.vyaw_setpoint = 0
           
            # preparing information to be passed
            self.gains.data = [
                        indicator,
                        self.vx,
                        self.vy,
                        self.vz,
                        self.vyaw
                    ]
            
            # publishing information
            if self.current_mode == "velocity":
                self.pub9.publish(self.gains)
                rospy.loginfo("Setting velocity to zero")
            else:
                rospy.logwarn("velocity mode is not active")

        except:
            rospy.logerr("Encountered an error in set_zero_velocity")


    def submit_velocity_setpoint(self):
        try:
            # indicator tells the subscriber what information is being passed 
            indicator = 5
            

            # setting velocities
            self.vx = self.get_user_input(self.velocity_mode_dict["vx"])
            self.vy = self.get_user_input(self.velocity_mode_dict["vy"])
            self.vz = self.get_user_input(self.velocity_mode_dict["vz"])
            self.vyaw = self.get_user_input(self.velocity_mode_dict["vyaw"])

            # preparing information to be passed
            self.gains.data = [
                        indicator,
                        self.vx,
                        self.vy,
                        self.vz,
                        self.vyaw
                    ]

            # publishing information
            if self.current_mode == "velocity":
                self.pub9.publish(self.gains)
                rospy.loginfo("Setting velocity")
                rospy.loginfo(f"vyaw setpoint gui = {self.vyaw}")
            else:
                rospy.logwarn("velocity mode is not active")
                
        except:
            rospy.logerr("Encountered an error in submit_velocity_setpoint")


    def set_zero_pwm(self):
        try:
            # indicator tells the subscriber what information is being passed 
            indicator = 7

            # setting PWM to zero
            self.x_pwm = 0
            self.y_pwm = 0
            self.z_pwm = 500 
            self.yaw_pwm  = 0
        
            # preparing information to be passed
            self.gains.data = [
                    indicator,
                    self.x_pwm,
                    self.y_pwm,
                    self.z_pwm,
                    self.yaw_pwm
                ]

            if self.current_mode == "manual pwm":
                self.pub9.publish(self.gains)
                rospy.loginfo("Sentting PWM to zero")
        
            else:
                rospy.logwarn("manual pwm mode is not active")
        except:
            rospy.logerr("Encountered an error in set_zero_pwm")
      

    def submit_pwm_setpoint(self):
        try:
            # indicator tells the subscriber what information is being passed 
            indicator = 7

            # setting PWM to zero
            self.x_pwm = self.get_user_input(self.pwm_mode_dict["x_pwm"])
            self.y_pwm = self.get_user_input(self.pwm_mode_dict["y_pwm"])
            self.z_pwm = self.get_user_input(self.pwm_mode_dict["z_pwm"])
            self.yaw_pwm  = self.get_user_input(self.pwm_mode_dict["yaw_pwm"])
        
            # quick work around for the differing scale and for when z is left blank
            if self.z_pwm == 0.0:
                self.z_pwm  = 500
            
            # preparing information to be passed
            self.gains.data = [
                    indicator,
                    self.x_pwm,
                    self.y_pwm,
                    self.z_pwm,
                    self.yaw_pwm
                ]

            if self.current_mode == "manual pwm":
                self.pub9.publish(self.gains)
                rospy.loginfo("Sending PWM setpoints")
            else:
                rospy.logwarn("manual pwm mode is not active")
        except:
            rospy.logerr("Encountered an error in submit_pwm_setpoint")
    

    def submit_target_depth(self):

        try:
            # indicator tells the subscriber what information is being passed
            indicator = 6

            # getting target depth from the entry field based on which mode is active
            if self.current_mode == "velocity":
                self.target_depth = self.get_user_input(self.velocity_mode_dict["target_depth"])
            elif self.current_mode == "manual pwm":
                self.target_depth = self.get_user_input(self.pwm_mode_dict["target_depth"])
             

            # add else ifs for other modes here 
            
            # preparing information to be passed
            self.gains.data = [
                    indicator,
                    self.target_depth
                ]
            
            # publishing information
            self.pub9.publish(self.gains)
            rospy.loginfo(f"Setting target depth to {self.target_depth} m")
       
        except:
            rospy.logerr("Encountered an error in submit_target_depth")


    def submit_controller_gains(self,suffix):
        """ Extract values from entries and publish them """
        try:
            self.gains.data.clear()
            self.kp_x =  self.get_user_input(f"kp_x_entry_{suffix}")
            self.kd_x =  self.get_user_input(f"kd_x_entry_{suffix}")
            self.ki_x =  self.get_user_input(f"ki_x_entry_{suffix}")

            self.kp_y =  self.get_user_input(f"kp_y_entry_{suffix}")
            self.kd_y =  self.get_user_input(f"kd_y_entry_{suffix}")
            self.ki_y =  self.get_user_input(f"ki_y_entry_{suffix}")

            self.kp_z =  self.get_user_input(f"kp_z_entry_{suffix}")
            self.kd_z =  self.get_user_input(f"kd_z_entry_{suffix}")
            self.ki_z =  self.get_user_input(f"ki_z_entry_{suffix}")

            self.kp_yaw =  self.get_user_input(f"kp_yaw_entry_{suffix}")
            self.kd_yaw =  self.get_user_input(f"kd_yaw_entry_{suffix}")
            self.ki_yaw =  self.get_user_input(f"ki_yaw_entry_{suffix}")

            # indicator indicates which pid gains are bing changed without creating a new topic
            if suffix=='1':
                indicator = 1
                rospy.loginfo("Sending controller gains for position controller")
                              
                self.gains.data = [
                        indicator,
                        self.kp_x,
                        self.kd_x,
                        self.ki_x,
                        self.kp_y,
                        self.kd_y,
                        self.ki_y,
                        self.kp_z,
                        self.kd_z,
                        self.ki_z,
                        self.kp_yaw,
                        self.kd_yaw,
                        self.ki_yaw,
                    ]

            elif suffix == '2':
                indicator = 2
                rospy.loginfo("Sending controller gains for velocity controller")
          
                   
                self.gains.data = [
                        indicator,
                        self.kp_x,
                        self.kd_x,
                        self.ki_x,
                        self.kp_y,
                        self.kd_y,
                        self.ki_y,
                        self.kp_z,
                        self.kd_z,
                        self.ki_z,
                        self.kp_yaw,
                        self.kd_yaw,
                        self.ki_yaw,
                    ]  
      
            
            self.pub9.publish(self.gains)
           

        except ValueError as ve:
            rospy.logerr(f"Invalid input for controller gains: {ve}")
            # messagebox.showerror("Input Error", f"Invalid input for controller gains: {ve}")
        

    def submit_max_linear_velocity(self):
        try:
            indicator = 3
            rospy.loginfo("Sending max linear velocity")
            self.max_linear_velocity  = float(getattr(self,"max_linear_velocity_entry").get())
            self.gains.data = [
                    indicator,
                    self.max_linear_velocity
                    ]
            self.pub9.publish(self.gains)
        except:
            rospy.logerr("Encountered an error in submit_max_linear_velocity")
            

    def submit_max_angular_velocity(self):
        try:
            indicator = 9
            rospy.loginfo("Sending max angular velocity")
            self.max_angular_velocity  = float(getattr(self,"max_angular_velocity_entry").get())
            self.gains.data = [
                    indicator,
                    self.max_angular_velocity
                    ]
            self.pub9.publish(self.gains)
        except:
            rospy.logerr("Encountered an error in submit_max_linear_velocity")
        

    def submit_max_x_y_pwm(self):
        try:
            indicator = 4
            rospy.loginfo("Sending max pwm signal")
            self.max_pwm  = float(getattr(self,"x_y_pwm_entry").get())
            self.gains.data = [
                    indicator,
                    self.max_pwm
                    ]
            self.pub9.publish(self.gains)
        except: 
            rospy.logerr("Encountered an error in submit_max_x_y_pwm")
        


    #  Controller parameters window
    def open_controller_params_window(self):
       
        try:  
            self.controller_params_window.deiconify()  
            
        except:  
        
            # Create a new window
            self.controller_params_window = Toplevel(self.master)
            self.controller_params_window.title("controller params")
            # Position Controller Parameters
            self.position_controller_params = LabelFrame(self.controller_params_window, text="Position Controller Gains")
            self.position_controller_params.grid(row = 0, column=0, columnspan=1, pady=10, sticky="ew")
            self.create_pid_gains(self.position_controller_params,'1')

            # Velocity Controller Parameters
            self.velocity_controller_params = LabelFrame(self.controller_params_window, text="Velocity Controller Gains")
            self.velocity_controller_params.grid(row = 1, column=0, columnspan=1, pady=10, sticky="ew")
            self.create_pid_gains(self.velocity_controller_params,'2')


            # Saturation Parameters
            self.saturation_params = LabelFrame(self.controller_params_window, text="Saturation Parameters")
            self.saturation_params.grid(row = 2, column=0, columnspan=1, pady=10, sticky="ew")
            self.create_saturation_params(self.saturation_params)

            # self.create_pid_gains(self.PWM_pid_gains,'2')

            # Button to return to the main window
            self.controller_params_window_back_button = Button(self.controller_params_window, text="Close Window", command=self.controller_params_window.destroy)
            self.controller_params_window_back_button.grid(row = 3, column=0, sticky="ew", padx=2, pady=2)



    # Other

    def position_callback(self, msg):
        self.current_pose = msg
        self.current_pose_transform = self.get_current_pose_transform()
  
    def get_current_pose_transform(self):
        # convert current pose to transformation matrix relative in NED frame
        trans_matrix = translation_matrix([self.current_pose.pose.pose.position.x,self.current_pose.pose.pose.position.y,self.current_pose.pose.pose.position.z])
        roll, pitch, yaw = euler_from_quaternion([self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w])
        rot_matrix = euler_matrix(roll,pitch,yaw)
        # transform  = concatenate_matrices(rot_matrix, trans_matrix)
        transform  = concatenate_matrices(trans_matrix,rot_matrix)
        # rospy.loginfo(transform)

        return transform
        # rospy.loginfo(self.current_pose_transform )

    def create_waypoint_fields(self, frame, suffix):
        """ Helper function to create labeled entry widgets """
        x_label = Label(frame, text="X:")
        x_label.grid(row=0, column=0, padx=5, pady=5)
        setattr(self, f"x{suffix}_entry", Entry(frame, width = 10))
        getattr(self, f"x{suffix}_entry").grid(row=0, column=1, padx=5, pady=5)

        y_label = Label(frame, text="Y:")
        y_label.grid(row=1, column=0, padx=5, pady=5)
        setattr(self, f"y{suffix}_entry", Entry(frame, width = 10))
        getattr(self, f"y{suffix}_entry").grid(row=1, column=1, padx=5, pady=5)

        z_label = Label(frame, text="Z:")
        z_label.grid(row=2, column=0, padx=5, pady=5)
        setattr(self, f"z{suffix}_entry", Entry(frame, width = 10))
        getattr(self, f"z{suffix}_entry").grid(row=2, column=1, padx=5, pady=5)

        # roll_label = Label(frame, text="Roll:")
        # roll_label.grid(row=3, column=0, padx=5, pady=5)
        # setattr(self, f"roll{suffix}_entry", Entry(frame, width = 10))
        # getattr(self, f"roll{suffix}_entry").grid(row=3, column=1, padx=5, pady=5)

        # pitch_label = Label(frame, text="Pitch:")
        # pitch_label.grid(row=4, column=0, padx=5, pady=5)
        # setattr(self, f"pitch{suffix}_entry", Entry(frame, width = 10))
        # getattr(self, f"pitch{suffix}_entry").grid(row=4, column=1, padx=5, pady=5)

        yaw_label = Label(frame, text="Yaw:")
        yaw_label.grid(row=3, column=0, padx=5, pady=5)
        setattr(self, f"yaw{suffix}_entry", Entry(frame, width = 10))
        getattr(self, f"yaw{suffix}_entry").grid(row=3, column=1, padx=5, pady=5)

        visualize_button = Button(frame, text=f"Visualize Waypoint", command=lambda: self.visualize_potential_waypoint(suffix))
        visualize_button.grid(row=4, column=1, columnspan=2, pady=5)

        submit_button = Button(frame, text=f"Submit Waypoint", command=lambda: self.submit_waypoint(suffix))
        submit_button.grid(row=5, column=1, columnspan=2,pady=5)



    # need to subscribe to info about the current state or origin
    def set_home_pose(self):
        """ Sets the home position """
        # pose_msg = Pose()
        # pose_msg.position.x = 0.0
        # pose_msg.position.y = 0.0
        # pose_msg.position.z = 0.0

        # quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        # pose_msg.orientation.x = quaternion[0]
        # pose_msg.orientation.y = quaternion[1]
        # pose_msg.orientation.z = quaternion[2]
        # pose_msg.orientation.w = quaternion[3]
        self.home_pose.header.frame_id='NED'
        self.home_pose.pose = self.current_pose.pose.pose
        # self.pub1.publish(pose_msg)
        # self.goal_waypoints.poses.append(pose_msg)
        rospy.loginfo(f"Set home position as: {self.home_pose.pose}")

    def erase_waypoints(self):
        """ Erase the waypoints array """
        self.goal_waypoints.poses.clear()
        self.desired_path.poses.clear()

        roll, pitch, yaw = euler_from_quaternion([self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w])

        self.add_defined_waypoint(self.current_pose.pose.pose.position.x, 
                                  self.current_pose.pose.pose.position.y,
                                  self.current_pose.pose.pose.position.z,
                                  roll,
                                  pitch,
                                  yaw)
    
        self.start_waypoint_index = 0
        self.pub6.publish(self.start_waypoint_index)
        self.pub3.publish(self.goal_waypoints)
        self.pub4.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
    
        rospy.loginfo("Erased all waypoints, reset waypoint controller index, holding current position")
        # messagebox.showinfo("Success", "All waypoints have been erased.")

    def go_home(self):
        """ Clears waypoints and adds home position as the first waypoint and immediatley heads to home """
        self.erase_waypoints()
        self.goal_waypoints.poses.append(self.home_pose.pose)
        self.desired_path.poses.append(self.home_pose)

        self.pub3.publish(self.goal_waypoints)
        self.pub4.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
        rospy.loginfo("Reset waypoints to home position")
        # messagebox.showinfo("Success", "Waypoints have been cleared. Ready to head to home position.")

    def add_home(self):
        """ Adds the home postion as the next waypoint"""
        self.goal_waypoints.poses.append(self.home_pose.pose)
        self.desired_path.poses.append(self.home_pose)
        self.pub3.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
        rospy.loginfo(f"Added home waypoint to list:\n {self.home_pose}")
        
    def submit_waypoint(self, suffix):
        """ Extract values from entries and publish them """
        try:
            x = self.get_user_input(f"x{suffix}_entry")
            y = self.get_user_input(f"y{suffix}_entry")
            z = self.get_user_input(f"z{suffix}_entry")
            yaw = self.get_user_input(f"yaw{suffix}_entry")*(np.pi / 180)

            
            #     float(getattr(self, f"x{suffix}_entry").get())
            # y = float(getattr(self, f"y{suffix}_entry").get())
            # z = float(getattr(self, f"z{suffix}_entry").get())
            # roll = float(getattr(self, f"roll{suffix}_entry").get())*(np.pi / 180)
            # pitch = float(getattr(self, f"pitch{suffix}_entry").get())*(np.pi / 180)
            roll = 0
            pitch = 0
            yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)


            pose_msg = Pose()

            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            self.desired_path.header.frame_id = 'NED'
            if suffix == '1':
                pose_msg.position.x = x
                pose_msg.position.y = y
                pose_msg.position.z = z

                quaternion = quaternion_from_euler(roll, pitch, yaw)
                pose_msg.orientation.x = quaternion[0]
                pose_msg.orientation.y = quaternion[1]
                pose_msg.orientation.z = quaternion[2]
                pose_msg.orientation.w = quaternion[3]

                # update the transform for the waypoint
                trans_matrix = translation_matrix([x,y,z])
                rot_matrix = euler_matrix(roll,pitch,yaw)
                transform = concatenate_matrices(rot_matrix, trans_matrix)


            elif suffix == '2':
                                
                # convert relative transdorm to global transform
                trans_matrix = translation_matrix([x,y,z])
                rot_matrix = euler_matrix(roll,pitch,yaw)
                transform = concatenate_matrices(trans_matrix, rot_matrix) 
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

                # update the last waypoint transform
                # self.last_waypoint_transform = transform

            elif suffix == '3':
             
                pose_msg = self.last_waypoint_rel_to_current_state_visualized
                transform = self.last_waypoint_rel_to_current_state_visualized_transform 

                # # storing the transformation for last waypoint added
                # trans_matrix = translation_matrix([pose_msg.position.x,pose_msg.position.y,pose_msg.position.z])
                # rot_matrix = quaternion_matrix([pose_msg.orientation.x, pose_msg.orientation.y,pose_msg.orientation.z,pose_msg.orientation.w])
                # transform = concatenate_matrices(rot_matrix, trans_matrix)

                

            # update the last waypoint transform
            self.last_waypoint_transform = transform

        
            # adding the waypoint to the waypoint array and desired path 
            pose_stamped_msg.pose = pose_msg
            self.goal_waypoints.poses.append(pose_msg)

            # testing the path message
            self.desired_path.poses.append(pose_stamped_msg)
            rospy.loginfo(f"Added waypoint to list:\n {pose_msg}")

            # Publishing waypoints array to the visualizer
            self.pub3.publish(self.goal_waypoints)
            self.pub5.publish(self.desired_path)
            # rospy.loginfo("Added waypoint to aypoint list")


        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint {suffix}: {ve}")
            # messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")

    def visualize_potential_waypoint(self, suffix):
        """Visualize the single waypoint just entered"""
        try:
            # x = float(getattr(self, f"x{suffix}_entry").get())
            # y = float(getattr(self, f"y{suffix}_entry").get())
            # z = float(getattr(self, f"z{suffix}_entry").get())
            # # roll = float(getattr(self, f"roll{suffix}_entry").get())*(np.pi / 180)
            # # pitch = float(getattr(self, f"pitch{suffix}_entry").get())*(np.pi / 180)
            roll = 0
            pitch =0
            # yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)
            x = self.get_user_input(f"x{suffix}_entry")
            y = self.get_user_input(f"y{suffix}_entry")
            z = self.get_user_input(f"z{suffix}_entry")
            yaw = self.get_user_input(f"yaw{suffix}_entry")*(np.pi / 180)

            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            if suffix == '1':
                # pose_stamped_msg.header.frame_id = 'NED'
                pose_msg.position.x = x
                pose_msg.position.y = y
                pose_msg.position.z = z

                quaternion = quaternion_from_euler(roll, pitch, yaw)
                pose_msg.orientation.x = quaternion[0]
                pose_msg.orientation.y = quaternion[1]
                pose_msg.orientation.z = quaternion[2]
                pose_msg.orientation.w = quaternion[3]

                # # update the transform for the waypoint
                # trans_matrix = translation_matrix([x,y,z])
                # rot_matrix = euler_matrix(roll,pitch,yaw)
                # transform = concatenate_matrices(rot_matrix, trans_matrix)

            elif suffix == '2':
                # pose_stamped_msg.header.frame_id = 'NED'
                # convert relative transform to global transform
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
            
        
            elif suffix == '3':
            
                # convert relative transform relative to body frame to Ned frame
                trans_matrix = translation_matrix([x,y,z])
                rot_matrix = euler_matrix(roll,pitch,yaw)    
                transform = concatenate_matrices(trans_matrix, rot_matrix)            
                # transform = concatenate_matrices(rot_matrix,trans_matrix) 
                # transform = concatenate_matrices(self.get_current_pose_transform(), rot_matrix, trans_matrix)
                # transform = concatenate_matrices(self.current_pose_transform, transform)
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
            rospy.loginfo(f"Visualized single waypoint: {pose_msg}")

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint {suffix}: {ve}")


            # messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")
    
    def visualize_waypoints(self):
        """ Publish the waypoints list to the plot when the button is pressed """
        # if not self.goal_waypoints.poses:
        #     rospy.logwarn("Waypoint array is empty")
        # else:
        #     # rospy.loginfo("waypoints: \n" + self.goal_waypoints.poses)
        #     self.pub3.publish(self.goal_waypoints)
        #     self.pub5.publish(self.desired_path)
        #     rospy.loginfo("Published goal waypoints array and path for visualizing")
        #     # rospy.loginfo("waypoints: \n" + self.goal_waypoints.poses)
        # self.pub3.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
        rospy.loginfo("Published goal waypoints array and path for visualizing")


    # General functions 
    def publish_goal_waypoints(self):
        self.pub4.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
        rospy.loginfo("sending waypoints")
        
    def add_defined_waypoint(self,x,y,z,roll,pitch,yaw):
        # x=0,y=0,z=0,roll=0,pitch=0,yaw =0):
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
        
        # adding the waypoint to the waypoint array and desired path 
        pose_stamped_msg.pose = pose_msg
        self.goal_waypoints.poses.append(pose_msg)

        # Publishing waypoints array to the visualizer
        self.desired_path.poses.append(pose_stamped_msg)

    def generate_square(self):
        # length, width, depth
        l, w, d = 4, 4, self.current_pose.pose.pose.position.z

        self.add_defined_waypoint(x=0,y=0,z=d,roll=0,pitch=0,yaw =0)
        self.add_defined_waypoint(x=l,y=0,z=d,roll=0,pitch=0,yaw =0)
        self.add_defined_waypoint(x=l,y=0,z=d,roll=0,pitch=0,yaw =np.pi/2)
        self.add_defined_waypoint(x=l,y=w,z=d,roll=0,pitch=0,yaw =np.pi/2)
        self.add_defined_waypoint(x=l,y=w,z=d,roll=0,pitch=0,yaw =np.pi)
        self.add_defined_waypoint(x=0,y=w,z=d,roll=0,pitch=0,yaw =np.pi)
        self.add_defined_waypoint(x=0,y=w,z=d,roll=0,pitch=0,yaw =3*np.pi/2)
        self.add_defined_waypoint(x=0,y=0,z=d,roll=0,pitch=0,yaw =3*np.pi/2)
        self.add_defined_waypoint(x=0,y=0,z=d,roll=0,pitch=0,yaw =0)
        self.add_defined_waypoint(x=0,y=0,z=0,roll=0,pitch=0,yaw =0)

        # self.add_defined_waypoint(x=0,y=0,z=0,roll=0,pitch=0,yaw =0)
        # self.add_defined_waypoint(x=0,y=0,z=d,roll=0,pitch=0,yaw =0)
        # self.add_defined_waypoint(x=l,y=0,z=d,roll=0,pitch=0,yaw =0)
        # self.add_defined_waypoint(x=l,y=0,z=d,roll=0,pitch=0,yaw =np.pi/2)
        # self.add_defined_waypoint(x=l,y=w,z=d,roll=0,pitch=0,yaw =np.pi/2)
        # self.add_defined_waypoint(x=l,y=w,z=d,roll=0,pitch=0,yaw =np.pi)
        # self.add_defined_waypoint(x=0,y=w,z=d,roll=0,pitch=0,yaw =np.pi)
        # self.add_defined_waypoint(x=0,y=w,z=d,roll=0,pitch=0,yaw =3*np.pi/2)
        # self.add_defined_waypoint(x=0,y=0,z=d,roll=0,pitch=0,yaw =3*np.pi/2)
        # self.add_defined_waypoint(x=0,y=0,z=d,roll=0,pitch=0,yaw =0)
        # self.add_defined_waypoint(x=0,y=0,z=0,roll=0,pitch=0,yaw =0)

        rospy.loginfo("Publishing square waypoints")
        self.pub3.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
    
        # no longer needed
    
    def toggle_hold_pose(self):

        if self.hold_pose:
            self.hold_pose = False
            rospy.loginfo("Continuing to waypoints")
            self.pub7.publish(self.hold_pose)
        
        elif not self.hold_pose:
            self.hold_pose = True
            rospy.loginfo("Requested position hold")
            self.pub7.publish(self.hold_pose)

            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'NED'
            pose_stamped_msg.pose = self.current_pose.pose.pose
    
            self.pub8.publish(pose_stamped_msg)

def main():
    rospy.init_node('waypoint_gui')
    root = Tk()
    gui = WaypointGui(root)
    
    while not rospy.is_shutdown():
        root.mainloop()
    # root.destroy()


if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
