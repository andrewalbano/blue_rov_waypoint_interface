#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped,PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, euler_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix, euler_from_quaternion
from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox, Frame


class WaypointGui:
    def __init__(self, master):
        # Initialize variables 
        self.goal_waypoints = PoseArray()
        self.goal_waypoints.header.frame_id = 'map_frame'
        
        self.last_waypoint_transform = np.eye(4)
        self.last_waypoint = Pose()
        self.home_pose = Pose()

        self.current_pose = PoseWithCovarianceStamped()
        # self.current_pose_transform = np.eye(4)
        
        # this stores the last waypoint visualized relative to the current state at the time of visualizing
        self.last_waypoint_rel_to_current_state_visualized = Pose()
        


        # initialize the GUI
        self.master = master
        self.master.title("Manage Waypoints")

        # Create the main container
        self.main_frame = Frame(master)
        self.main_frame.pack(padx=10, pady=10, fill="both", expand=True)
        
        # Create frames for waypoint fields to be positioned side by side. Waypoint fields are the multi row input frames allowing for poses to be added to the gui
        # self.waypoint_frame = LabelFrame(self.waypoint_frame, text="text")
        # self.waypoints_frame = Frame(self.main_frame)
        self.waypoints_frame = LabelFrame(self.main_frame, text="Add Waypoints")
        self.waypoints_frame.grid(row=0, column=0, columnspan=2, pady=10, sticky="ew")
        
        # generate individual waypoint fields
        self.waypoint1_frame = LabelFrame(self.waypoints_frame, text="Add a global waypoint to the waypoint list")
        self.waypoint1_frame.grid(row=1, column=0, padx=5, pady=5, sticky="nsew")

        self.waypoint2_frame = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to the last waypoint added")
        self.waypoint2_frame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")

        self.waypoint3_frame = LabelFrame(self.waypoints_frame, text="Add a waypoint relative to the current pose")
        self.waypoint3_frame.grid(row=3, column=0, padx=5, pady=5, sticky="nsew")
        
        # # Create frame for generating a square pattern in x-y plane below the waypoint fields
        # self.square_pattern_frame = LabelFrame(self.main_frame, text="Generate Square Pattern in x-y plane at a global depth")
        # self.square_pattern_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Create fields for waypoint 1 and 2
        self.create_waypoint_fields(self.waypoint1_frame, '1')
        self.create_waypoint_fields(self.waypoint2_frame, '2')
        self.create_waypoint_fields(self.waypoint3_frame, '3')


        # Create a separate frame for buttons and put it beside the fields frame
        # self.button_frame = Frame(self.main_frame)
        self.button_frame = LabelFrame(self.main_frame, text = "Manage waypoints")
        self.button_frame.grid(row=0, column=3, columnspan=2, pady=10, sticky="ew")

        # Add buttons beside the waypoint frames
        self.home_button = Button(self.button_frame, text="Set Home Position", command=self.set_home_position)
        self.home_button.grid(row=0, column=0, columnspan=2, sticky="ew", pady=2)

        self.add_home_button = Button(self.button_frame, text="Add Home", command=self.add_home)
        self.add_home_button.grid(row=1, column=0, columnspan=2, sticky="ew", pady=2)

        self.go_home_button = Button(self.button_frame, text="Go Home", command=self.go_home)
        self.go_home_button.grid(row=2, column=0, columnspan=2, sticky="ew", pady=2)

        self.erase_waypoints_button = Button(self.button_frame, text="Erase Waypoints", command=self.erase_waypoints)
        self.erase_waypoints_button.grid(row=3, column=0, columnspan=2, sticky="ew", pady=2)

        self.publish_goal_waypoints = Button(self.button_frame, text="Visualize Waypoint List", command=self.visualize_waypoints)
        self.publish_goal_waypoints.grid(row=4, column=0, columnspan=2, sticky="ew", pady=2)

        # Initialize ROS node for publishing
        self.pub1 = rospy.Publisher('new_pose_visualization', PoseStamped, queue_size=10)
        self.pub3 = rospy.Publisher("waypoint_plot_visualization", PoseArray, queue_size=10)

        # Initialize subscriber
        # Subscribe to the local position topic
        self.sub1 = rospy.Subscriber('current_state_test', PoseWithCovarianceStamped, self.position_callback)
    
    def position_callback(self, msg):
        self.current_pose = msg
        self.current_pose_transform = self.get_current_pose_transform()
  
    def get_current_pose_transform(self):
        # convert current pose to transformation matrix relative to map frame
        trans_matrix = translation_matrix([self.current_pose.pose.pose.position.x,self.current_pose.pose.pose.position.y,self.current_pose.pose.pose.position.z])
        roll, pitch, yaw = euler_from_quaternion([self.current_pose.pose.pose.orientation.x, self.current_pose.pose.pose.orientation.y, self.current_pose.pose.pose.orientation.z, self.current_pose.pose.pose.orientation.w])
        rot_matrix = euler_matrix(roll,pitch,yaw)
        transform  = concatenate_matrices(rot_matrix, trans_matrix)
        return transform
        # rospy.loginfo(self.current_pose_transform )

    def create_waypoint_fields(self, frame, suffix):
        # fields = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        # self.entries = {}

        # for i, field in enumerate(fields):
        #     label = Label(frame, text=f"{field.capitalize()}:")
        #     label.grid(row=i, column=0, padx=5, pady=5)
        #     entry = Entry(frame)
        #     entry.grid(row=i, column=1, padx=5, pady=5)
        #     self.entries[f"{field}{suffix}"] = entry
        
        # submit_button = Button(frame, text=f"Submit Waypoint {suffix}", command=lambda: self.submit_waypoint(suffix))
        # submit_button.grid(row=len(fields), column=0, columnspan=2, pady=10)
        """ Helper function to create labeled entry widgets """
        x_label = Label(frame, text="X:")
        x_label.grid(row=0, column=0, padx=5, pady=5)
        setattr(self, f"x{suffix}_entry", Entry(frame))
        getattr(self, f"x{suffix}_entry").grid(row=0, column=1, padx=5, pady=5)

        y_label = Label(frame, text="Y:")
        y_label.grid(row=1, column=0, padx=5, pady=5)
        setattr(self, f"y{suffix}_entry", Entry(frame))
        getattr(self, f"y{suffix}_entry").grid(row=1, column=1, padx=5, pady=5)

        z_label = Label(frame, text="Z:")
        z_label.grid(row=2, column=0, padx=5, pady=5)
        setattr(self, f"z{suffix}_entry", Entry(frame))
        getattr(self, f"z{suffix}_entry").grid(row=2, column=1, padx=5, pady=5)

        roll_label = Label(frame, text="Roll:")
        roll_label.grid(row=3, column=0, padx=5, pady=5)
        setattr(self, f"roll{suffix}_entry", Entry(frame))
        getattr(self, f"roll{suffix}_entry").grid(row=3, column=1, padx=5, pady=5)

        pitch_label = Label(frame, text="Pitch:")
        pitch_label.grid(row=4, column=0, padx=5, pady=5)
        setattr(self, f"pitch{suffix}_entry", Entry(frame))
        getattr(self, f"pitch{suffix}_entry").grid(row=4, column=1, padx=5, pady=5)

        yaw_label = Label(frame, text="Yaw:")
        yaw_label.grid(row=5, column=0, padx=5, pady=5)
        setattr(self, f"yaw{suffix}_entry", Entry(frame))
        getattr(self, f"yaw{suffix}_entry").grid(row=5, column=1, padx=5, pady=5)

        visualize_button = Button(frame, text=f"Visualize Waypoint", command=lambda: self.visualize_potential_waypoint(suffix))
        visualize_button.grid(row=6, column=0, columnspan=2, pady=10)

        submit_button = Button(frame, text=f"Submit Waypoint", command=lambda: self.submit_waypoint(suffix))
        submit_button.grid(row=6, column=2, columnspan=2,pady=10)

    
    # need to subscribe to info about the current state or origin
    def set_home_position(self):
        """ Sets the home position """
        pose_msg = Pose()
        pose_msg.position.x = 0.0
        pose_msg.position.y = 0.0
        pose_msg.position.z = 0.0

        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        pose_msg.orientation.x = quaternion[0]
        pose_msg.orientation.y = quaternion[1]
        pose_msg.orientation.z = quaternion[2]
        pose_msg.orientation.w = quaternion[3]

        self.home_pose = pose_msg
        # self.pub1.publish(pose_msg)
        self.goal_waypoints.poses.append(pose_msg)
        rospy.loginfo("Set home position and published")

    def erase_waypoints(self):
        """ Erase the waypoints array """
        self.goal_waypoints.poses.clear()
        rospy.loginfo("Erased all waypoints")
        messagebox.showinfo("Success", "All waypoints have been erased.")

    def go_home(self):
        """ Clears waypoints and adds home position as the first waypoint """
        self.goal_waypoints.poses.clear()
        self.goal_waypoints.poses.append(self.home_pose)
        rospy.loginfo("Reset waypoints to home position")
        messagebox.showinfo("Success", "Waypoints have been reset to home position.")

    def add_home(self):
        """ Adds the home postion as the next waypoint"""
        self.goal_waypoints.poses.append(self.home_pose)
        rospy.loginfo(f"Added home waypoint to list:\n {self.home_pose}")
        
    # need to add conversion to clobal frame if relative waypoint is given
    def submit_waypoint(self, suffix):
        """ Extract values from entries and publish them """
        try:
            x = float(getattr(self, f"x{suffix}_entry").get())
            y = float(getattr(self, f"y{suffix}_entry").get())
            z = float(getattr(self, f"z{suffix}_entry").get())
            roll = float(getattr(self, f"roll{suffix}_entry").get())*(np.pi / 180)
            pitch = float(getattr(self, f"pitch{suffix}_entry").get())*(np.pi / 180)
            yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)


            pose_msg = Pose()

            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'map_frame'
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
                self.last_waypoint_transform = concatenate_matrices(rot_matrix, trans_matrix)


            elif suffix == '2':
                                
                # convert relative transdorm to global transform
                trans_matrix = translation_matrix([x,y,z])
                rot_matrix = euler_matrix(roll,pitch,yaw)
                transform = concatenate_matrices(self.last_waypoint_transform, rot_matrix, trans_matrix)

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
                self.last_waypoint_transform = transform

            elif suffix == '3':
                pose_msg = self.last_waypoint_rel_to_current_state_visualized 

            pose_stamped_msg.pose = pose_msg

            self.goal_waypoints.poses.append(pose_msg)
            rospy.loginfo(f"Added waypoint to list:\n {pose_msg}")

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint {suffix}: {ve}")
            messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")

    def visualize_potential_waypoint(self, suffix):
        """Visualize the single waypoint just entered"""
        try:
            x = float(getattr(self, f"x{suffix}_entry").get())
            y = float(getattr(self, f"y{suffix}_entry").get())
            z = float(getattr(self, f"z{suffix}_entry").get())
            roll = float(getattr(self, f"roll{suffix}_entry").get())*(np.pi / 180)
            pitch = float(getattr(self, f"pitch{suffix}_entry").get())*(np.pi / 180)
            yaw = float(getattr(self, f"yaw{suffix}_entry").get())*(np.pi / 180)


            pose_msg = Pose()
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'map_frame'
            if suffix == '1':
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
                                
                # convert relative transform to global transform
                trans_matrix = translation_matrix([x,y,z])
                rot_matrix = euler_matrix(roll,pitch,yaw)
                transform = concatenate_matrices(self.last_waypoint_transform, rot_matrix, trans_matrix)

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
                #  if we do not have a body frame to use then this should work other wise transform it using this 
                # rospy.loginfo(self.current_pose)
                # rospy.loginfo(self.get_current_pose_transform())
                  
            
                # convert relative transform relative to body frame to  map frame
                trans_matrix = translation_matrix([x,y,z])
                rot_matrix = euler_matrix(roll,pitch,yaw)               
                transform = concatenate_matrices(self.get_current_pose_transform(), rot_matrix, trans_matrix)

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
            

                #  if we do have a body frame that gets publish i could define pose relative to body_frame
                # pose_stamped_msg.header.frame_id = 'body_frame'
                # pose_msg.position.x = x
                # pose_msg.position.y = y
                # pose_msg.position.z = z

                # quaternion = quaternion_from_euler(roll, pitch, yaw)
                # pose_msg.orientation.x = quaternion[0]
                # pose_msg.orientation.y = quaternion[1]
                # pose_msg.orientation.z = quaternion[2]
                # pose_msg.orientation.w = quaternion[3]

              
            # formatting for rviz visualization
            pose_stamped_msg.pose = pose_msg
           

            self.pub1.publish(pose_stamped_msg)
            rospy.loginfo(f"Visualized single waypoint: {pose_msg}")

        except ValueError as ve:
            rospy.logerr(f"Invalid input for waypoint {suffix}: {ve}")


            messagebox.showerror("Input Error", f"Invalid input for waypoint: {ve}")
    
    def visualize_waypoints(self):
        """ Publish the waypoints list to the plot when the button is pressed """
        if not self.goal_waypoints.poses:
            rospy.logwarn("Waypoint array is empty")
        else:
            # rospy.loginfo("waypoints: \n" + self.goal_waypoints.poses)
            self.pub3.publish(self.goal_waypoints)
            rospy.loginfo("Published goal waypoints array")


    # def square(self, depth)
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
