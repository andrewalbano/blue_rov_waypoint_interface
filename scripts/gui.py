#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped,PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, euler_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, euler_from_quaternion
from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox, Frame
from std_msgs.msg import Bool,Int8,Float32MultiArray
from nav_msgs.msg import Path

#  should consider switching things from pose to poseStamped
class WaypointGui:  
    def __init__(self, master):
        # Initialize variables 
        self.kp_xy = 0.0
        self.kd_xy = 0.0
        self.ki_xy = 0.0

        self.kp_z = 0.0
        self.kd_z = 0.0
        self.ki_z = 0.0

        self.kp_yaw = 0.0
        self.kd_yaw = 0.0
        self.ki_yaw = 0.0

        self.gains = Float32MultiArray()
        self.gains.data = [
            0,
            self.kp_xy,
            self.kd_xy,
            self.ki_xy,
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
        self.master.title("Manage Waypoints")

        # Create the main container
        self.main_frame = Frame(master)
        self.main_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        
        # Create frames for waypoint fields to be positioned side by side. Waypoint fields are the multi row input frames allowing for poses to be added to the gui
        # self.waypoint_frame = LabelFrame(self.waypoint_frame, text="text")
        # self.waypoints_frame = Frame(self.main_frame)

        # Major Grid 
    
        # Main_frame column 1
        self.waypoints_frame = LabelFrame(self.main_frame, text="Add Waypoints")
        self.waypoints_frame.grid(row = 0, column=0, columnspan=2, pady=10, sticky="ew")
        
        # # add waypoints frame 
        # # add waypoints frame row 1
        # self.add_waypoint_frame_1 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nNED frame")
        # self.add_waypoint_frame_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        # # add waypoints frame row 1 column 2
        # self.add_waypoint_frame_2 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe last waypoint added")
        # self.add_waypoint_frame_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        # # add waypoints frame row 2 column 1
        # self.add_waypoint_frame_3= LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe current pose")
        # self.add_waypoint_frame_3.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")

        # # add waypoints frame row 2 column 2
        # self.pid_gains= LabelFrame(self.waypoints_frame, text="PID Gains")
        # self.pid_gains.grid(row=2, column=1, padx=5, pady=5, sticky="nsew")

        # # add waypoints frame row 2 column 2
        # # currently empty 

        # add waypoints frame 
        # add waypoints frame row 1
        self.add_waypoint_frame_1 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nNED frame")
        self.add_waypoint_frame_1.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        # add waypoints frame row 1 column 2
        self.add_waypoint_frame_2 = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe last waypoint added")
        self.add_waypoint_frame_2.grid(row=0, column=1, padx=5, pady=5, sticky="nsew")
        # add waypoints frame row 2 column 1
        self.add_waypoint_frame_3= LabelFrame(self.waypoints_frame, text="Add a waypoint relative to \nthe current pose")
        self.add_waypoint_frame_3.grid(row=0, column=2, padx=5, pady=5, sticky="nsew")

        # add waypoints frame row 2 column 1
        self.position_pid_gains= LabelFrame(self.waypoints_frame, text="Position PID Gains")
        self.position_pid_gains.grid(row=1, column=0, columnspan = 2, padx=5, pady=5, sticky="nsew")

        # add waypoints frame row 2 column 2
        self.params = LabelFrame(self.waypoints_frame, text="Other Params")
        self.params.grid(row=1, column=2, columnspan = 2, padx=5, pady=5, sticky="nsew")

        # add waypoints frame row 3 column 1
        self.PWM_pid_gains= LabelFrame(self.waypoints_frame, text="PWM PID Gains")
        self.PWM_pid_gains.grid(row=2, column=0, columnspan = 2, padx=5, pady=5, sticky="nsew")

        # add waypoints frame row 2 column 2
        self.velocity_testing = LabelFrame(self.waypoints_frame, text="velocity setpoint testing")
        self.velocity_testing.grid(row=2, column=2, columnspan = 2, padx=5, pady=5, sticky="nsew")


    


        # Create fields for waypoint 1 and 2
        self.create_waypoint_fields(self.add_waypoint_frame_1, '1')
        self.create_waypoint_fields(self.add_waypoint_frame_2, '2')
        self.create_waypoint_fields(self.add_waypoint_frame_3, '3')
        self.create_pid_gains(self.position_pid_gains,'1')
        self.create_pid_gains(self.PWM_pid_gains,'2')
        self.create_params(self.params,'3')
        self.create_velocity_setpoint(self.velocity_testing,'4')





        # Add buttons beside the waypoint frames
        # Main Frame row 2
        row_index = 0
        column_index = 0
        column_span = 2
        
        # Create a separate frame for buttons and put it beside the fields frame
        # self.motion_controller_frame = Frame(self.main_frame)
        self.motion_controller_frame = LabelFrame(self.main_frame, text = "Motion Controller")
        self.motion_controller_frame.grid(row=1, column=0, columnspan=2, pady=10, sticky="ew")

        # Add buttons beside the waypoint frames
        # main frame row 2
        row_index = 0
        column_index = 0
        column_span = 2
       
        # column 1
        self.toggle_motion_controller = Button(self.motion_controller_frame, text="Activate controller", command=self.enable_motion_control)
        self.toggle_motion_controller.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2,  pady=2)
        row_index +=1


        self.go_home_button = Button(self.motion_controller_frame, text="Go home", command=self.go_home)
        self.go_home_button.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew",padx=2, pady=2)
        row_index +=1


        self.publish_to_controller= Button(self.motion_controller_frame, text="Update waypoints", command=self.publish_goal_waypoints)
        self.publish_to_controller.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1


        # column 2
        row_index = 0
        column_index += column_span

        self.disable_motion_controller = Button(self.motion_controller_frame, text="Disable controller", command=self.disable_motion_control)
        self.disable_motion_controller.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1


        self.home_button = Button(self.motion_controller_frame, text="Set home position", command=self.set_home_pose)
        self.home_button.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1

        self.erase_waypoints_button = Button(self.motion_controller_frame, text="Erase Waypoints", command=self.erase_waypoints)
        self.erase_waypoints_button.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1

        # column 3
        row_index = 0
        column_index += column_span

        self.pause= Button(self.motion_controller_frame, text="Toggle hold pose", command=self.toggle_hold_pose)
        self.pause.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1

        self.add_home_button = Button(self.motion_controller_frame, text="Add home position", command=self.add_home)
        self.add_home_button.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1 

        self.visualize_goal_waypoints = Button(self.motion_controller_frame, text="Visualize plan", command=self.visualize_waypoints)
        self.visualize_goal_waypoints.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew", padx=2, pady=2)
        row_index +=1

        # main frame row 3 column 1
        row_index = 0
        column_index = 0
        column_span = 2
        self.generate_waypoints_frame = LabelFrame(self.main_frame, text = "Generate test patterns")
        self.generate_waypoints_frame.grid(row=2, column=0, columnspan=2, pady=10, sticky="ew")
        
        # generate waypoints frame row 1 column 1
        row_index = 0
        column_index = 0
        column_span = 2

        self.make_square= Button(self.generate_waypoints_frame, text="Generate Square", command=self.generate_square)
        self.make_square.grid(row=row_index, column=column_index, columnspan=column_span, sticky="ew",  padx=2, pady=2)
        row_index +=1



        # Initialize ROS node for publishing
        # publishes the last waypoint pose in rviz
        self.pub1 = rospy.Publisher('new_pose_visualization', PoseStamped, queue_size=10)
        # publishes the controller state on or off
        self.pub2 = rospy.Publisher("motion_control_state",Bool, queue_size=10)

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




        # Initialize subscriber
        # Subscribe to the local position topic
        # self.sub1 = rospy.Subscriber('current_state_test', PoseWithCovarianceStamped, self.position_callback)
        self.sub1 = rospy.Subscriber('/dvl/local_position', PoseWithCovarianceStamped, self.position_callback)

        # self.disable_motion_control()   
    
    def position_callback(self, msg):
        self.current_pose = msg
        self.current_pose_transform = self.get_current_pose_transform()
  
    def get_current_pose_transform(self):
        # convert current pose to transformation matrix relative in NED frame
        trans_matrix = translation_matrix([self.current_pose.pose.pose.position.x,self.current_pose.pose.pose.position.y,self.current_pose.pose.pose.position.z])
        roll, pitch, yaw = euler_from_quaternion([self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w])
        rot_matrix = euler_matrix(roll,pitch,yaw)
        transform  = concatenate_matrices(rot_matrix, trans_matrix)
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

    def create_velocity_setpoint(self, frame, suffix):
        """ Helper function to create labeled entry widgets """


        # row 1 
        vx_label = Label(frame, text="x velocity (m/s)")
        vx_label.grid(row=0, column=0, padx=5, pady=5)
        setattr(self, f"vx_entry", Entry(frame, width = 10))
        getattr(self, f"vx_entry").grid(row=0, column=1, padx=5, pady=5)



        vy_label = Label(frame, text="y velocity (m/s)")
        vy_label.grid(row=1, column=0, padx=5, pady=5)
        setattr(self, f"vy_entry", Entry(frame, width = 10))
        getattr(self, f"vy_entry").grid(row=1, column=1, padx=5, pady=5)


        vz_label = Label(frame, text="z velocity (m/s)")
        vz_label.grid(row=2, column=0, padx=5, pady=5)
        setattr(self, f"vz_entry", Entry(frame, width = 10))
        getattr(self, f"vz_entry").grid(row=2, column=1, padx=5, pady=5)

        vyaw_label = Label(frame, text="angular velocity (degree/s)")
        vyaw_label.grid(row=3, column=0, padx=5, pady=5)
        setattr(self, f"vyaw_entry", Entry(frame, width = 10))
        getattr(self, f"vyaw_entry").grid(row=3, column=1, padx=5, pady=5)

        v_testing_toggle_label = Label(frame, text="on / off")
        v_testing_toggle_label.grid(row=4, column=0, padx=5, pady=5)
        setattr(self, f"v_testing_toggle_entry", Entry(frame, width = 10))
        getattr(self, f"v_testing_toggle_entry").grid(row=4, column=1, padx=5, pady=5)



        submit_velocity_button = Button(frame, text=f"Submit velocity setpoint", command=lambda: self.submit_gains(suffix))
        submit_velocity_button.grid(row= 5, column=0,pady=10)


    


    def create_params(self, frame, suffix):
        """ Helper function to create labeled entry widgets """
        # row 1 
        linear_velocity_label = Label(frame, text="max linear velocity (m/s)")
        linear_velocity_label.grid(row=0, column=0, padx=5, pady=5)

        setattr(self, f"max_linear_velocity_entry", Entry(frame, width = 10))
        getattr(self, f"max_linear_velocity_entry").grid(row=0, column=1, padx=5, pady=5)

        min_pwm_label = Label(frame, text="min pwm")
        min_pwm_label.grid(row=1, column=0, padx=5, pady=5)
        setattr(self, f"min_pwm_entry", Entry(frame, width = 10))
        getattr(self, f"min_pwm_entry").grid(row=1, column=1, padx=5, pady=5)

        max_pwm_label = Label(frame, text="max pwm")
        max_pwm_label.grid(row=2, column=0, padx=5, pady=5)
        setattr(self, f"max_pwm_entry", Entry(frame, width = 10))
        getattr(self, f"max_pwm_entry").grid(row=2, column=1, padx=5, pady=5)


        submit_params_button = Button(frame, text=f"Submit params", command=lambda: self.submit_gains(suffix))

        submit_params_button.grid(row= 3, column=0,pady=10)
    


    def create_pid_gains(self, frame,suffix):
        """ Helper function to create labeled entry widgets """
        # row 1 
        kp_label = Label(frame, text="kp")
        kp_label.grid(row=0, column=1, padx=5, pady=5)

        kd_label = Label(frame, text="kd")
        kd_label.grid(row=0, column=2, padx=5, pady=5)       

        ki_label = Label(frame, text="ki")
        ki_label.grid(row=0, column=3, padx=5, pady=5)  



        # row 2
        xy_label = Label(frame, text="xy")
        xy_label.grid(row=1, column=0, padx=5, pady=5)
        setattr(self, f"kp_xy_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kp_xy_entry_{suffix}").grid(row=1, column=1, padx=5, pady=5)
        setattr(self, f"kd_xy_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kd_xy_entry_{suffix}").grid(row=1, column=2, padx=5, pady=5)
        setattr(self, f"ki_xy_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"ki_xy_entry_{suffix}").grid(row=1, column=3, padx=5, pady=5)

         # row 3
        z_label = Label(frame, text="z")
        z_label.grid(row=2, column=0, padx=5, pady=5)
        setattr(self, f"kp_z_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kp_z_entry_{suffix}").grid(row=2, column=1, padx=5, pady=5)
        setattr(self, f"kd_z_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kd_z_entry_{suffix}").grid(row=2, column=2, padx=5, pady=5)
        setattr(self, f"ki_z_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"ki_z_entry_{suffix}").grid(row=2, column=3, padx=5, pady=5)

         # row 4
        yaw_label = Label(frame, text="yaw")
        yaw_label.grid(row=3, column=0, padx=5, pady=5)
        setattr(self, f"kp_yaw_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kp_yaw_entry_{suffix}").grid(row=3, column=1, padx=5, pady=5)
        setattr(self, f"kd_yaw_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"kd_yaw_entry_{suffix}").grid(row=3, column=2, padx=5, pady=5)
        setattr(self, f"ki_yaw_entry_{suffix}", Entry(frame, width = 10))
        getattr(self, f"ki_yaw_entry_{suffix}").grid(row=3, column=3, padx=5, pady=5)

        submit_gains_button = Button(frame, text=f"Submit gains", command=lambda: self.submit_gains(suffix))
        submit_gains_button.grid(row=4, column=3,pady=10)
    
    
    def submit_gains(self,suffix):
        """ Extract values from entries and publish them """
        try:
            self.gains.data.clear()

            # indicator indicates which pid gains are bing changed without creating a new topic
            if suffix=='1':
                indicator = 1
                rospy.loginfo("Sending new controller gains for position controller")
            elif suffix == '2':
                indicator = 2
                rospy.loginfo("Sending new controller gains for PWM controller")
            elif suffix == '3':
                indicator = 3
                rospy.loginfo("Sending new parameters")
                
            elif suffix == '4':
                self.v_testing_toggle  = getattr(self,"v_testing_toggle_entry").get()
                if self.v_testing_toggle =="on":
                    indicator = 4
                    rospy.loginfo("Invoking velocity controller testing mode and sending veloity setpoints")
                else: 
                    indicator = 5 # returns to normal mode
                    rospy.loginfo("Disabling velocity testing")

            if suffix  == '1' or suffix =='2':
                self.kp_xy = float(getattr(self, f"kp_xy_entry_{suffix}").get())
                self.kd_xy = float(getattr(self, f"kd_xy_entry_{suffix}").get())
                self.ki_xy = float(getattr(self, f"ki_xy_entry_{suffix}").get())
                
                self.kp_z = float(getattr(self, f"kp_z_entry_{suffix}").get())
                self.kd_z = float(getattr(self, f"kd_z_entry_{suffix}").get())
                self.ki_z = float(getattr(self, f"ki_z_entry_{suffix}").get())
                
                
                self.kp_yaw = float(getattr(self, f"kp_yaw_entry_{suffix}").get())
                self.kd_yaw = float(getattr(self, f"kd_yaw_entry_{suffix}").get())
                self.ki_yaw = float(getattr(self, f"ki_yaw_entry_{suffix}").get())
                
                self.gains.data = [
                        indicator,
                        self.kp_xy,
                        self.kd_xy,
                        self.ki_xy,
                        self.kp_z,
                        self.kd_z,
                        self.ki_z,
                        self.kp_yaw,
                        self.kd_yaw,
                        self.ki_yaw
                    ]

            elif suffix == '3':
                self.max_linear_velocity  = float(getattr(self,"max_linear_velocity_entry").get())
                self.min_pwm  = float(getattr(self,"min_pwm_entry").get())
                self.max_pwm  = float(getattr(self,"max_pwm_entry").get())
                self.gains.data = [
                        indicator,
                        self.max_linear_velocity,
                        self.min_pwm,
                        self.max_pwm
                    ]
                
            elif suffix == '4':
                self.vx  = float(getattr(self,"vx_entry").get())
                self.vy  = float(getattr(self,"vy_entry").get())
                self.vz  = float(getattr(self,"vz_entry").get())
                self.vyaw  = float(getattr(self,"vyaw_entry").get())*np.pi/180
            
                self.gains.data = [
                        indicator,
                        self.vx,
                        self.vy,
                        self.vz,
                        self.vyaw
                    ]
            
            
            self.pub9.publish(self.gains)


            # rospy.loginfo("Sending new controller gains")
            # rospy.loginfo(self.gains)
        except ValueError as ve:
            rospy.logerr(f"Invalid input for controller gains: {ve}")
            messagebox.showerror("Input Error", f"Invalid input for controller gains: {ve}")
        

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
        
    # need to add conversion to clobal frame if relative waypoint is given
    def submit_waypoint(self, suffix):
        """ Extract values from entries and publish them """
        try:
            x = float(getattr(self, f"x{suffix}_entry").get())
            y = float(getattr(self, f"y{suffix}_entry").get())
            z = float(getattr(self, f"z{suffix}_entry").get())
            # roll = float(getattr(self, f"roll{suffix}_entry").get())*(np.pi / 180)
            # pitch = float(getattr(self, f"pitch{suffix}_entry").get())*(np.pi / 180)
            roll =0
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
            rospy.loginfo("Published goal waypoints array")


        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint {suffix}: {ve}")
            messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")

    def visualize_potential_waypoint(self, suffix):
        """Visualize the single waypoint just entered"""
        try:
            x = float(getattr(self, f"x{suffix}_entry").get())
            y = float(getattr(self, f"y{suffix}_entry").get())
            z = float(getattr(self, f"z{suffix}_entry").get())
            # roll = float(getattr(self, f"roll{suffix}_entry").get())*(np.pi / 180)
            # pitch = float(getattr(self, f"pitch{suffix}_entry").get())*(np.pi / 180)
            roll = 0
            pitch =0
            yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)


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


            messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")
    
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


    # no longer needed
    def toggle_motion_control(self):

        if self.motion_control_state == 1:
            self.motion_control_state = 0
            rospy.loginfo("Disabled motion controller")
            self.pub2.publish(self.motion_control_state)
        elif self.motion_control_state == 0:
            self.motion_control_state = 1
            self.pub2.publish(self.motion_control_state)
            rospy.loginfo("Enabled motion controller")
            self.pub4.publish(self.goal_waypoints)
            self.pub5.publish(self.desired_path)

    def enable_motion_control(self):
        self.motion_control_state = True
        rospy.loginfo("Activating motion controller and sending waypoints")
        self.pub4.publish(self.goal_waypoints)
        self.pub5.publish(self.desired_path)
        self.pub2.publish(self.motion_control_state)

    def disable_motion_control(self):
        self.motion_control_state = False
        rospy.loginfo("Disabling motion controller")
        self.pub2.publish(self.motion_control_state)


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
        l, w, d = 5, 5, 0

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
    root.mainloop()

if __name__ == '__main__':
    
    try:
        main()
    except rospy.ROSInterruptException:
        pass
