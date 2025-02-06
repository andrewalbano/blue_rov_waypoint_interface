#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped,PoseStamped
from tf.transformations import quaternion_from_euler, quaternion_matrix, translation_matrix, euler_matrix, concatenate_matrices, quaternion_from_matrix, translation_from_matrix
from tkinter import Tk, Label, Entry, Button, LabelFrame, messagebox

class WaypointGui:
    def __init__(self, master):
        self.master = master
        self.master.title("Manage Waypoints")

        # global Frame convention is earth //need to draw these out with onur
        # self.earth_frame = np.eye(4)
        # self.map_frame = np.eye(4)
        # self.odom_frame = np.eye(4)
        # self.base_frame = np.eye(4)


        self.goal_waypoints = PoseArray()
        # self.goal_waypoints.header.frame_id = 'global_frame'
        self.goal_waypoints.header.frame_id = 'map_frame'


        self.last_waypoint_transform =np.eye(4)
        self.last_waypoint = Pose()
        self.home_pose = Pose()
        self.current_state = Pose()

        # Create frames for adding global waypoint
        self.waypoint1_frame = LabelFrame(master, text="Add a global waypoint to the waypoint list")
        self.waypoint1_frame.pack(padx=10, pady=10, fill="both", expand=True)

        self.waypoint2_frame = LabelFrame(master, text="Add a waypoint relative to the last waypoint added")
        self.waypoint2_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # Create frame for generating a square pattern in x-y plane
        self.square_pattern_frame = LabelFrame(master, text="Generate Square Pattern in x-y plane at a global depth")
        self.square_pattern_frame.pack(padx=10, pady=10, fill="both", expand=True)

        
        # self.waypoint3_frame = LabelFrame(master, text="Add a waypoint relative to the current state")
        # self.waypoint3_frame.pack(padx=10, pady=10, fill="both", expand=True)

        # Create fields for waypoint 1 and 2
        self.create_waypoint_fields(self.waypoint1_frame, '1')
        self.create_waypoint_fields(self.waypoint2_frame, '2')
        
        # self.create_waypoint_fields(self.waypoint3_frame, '2')

        # Add Set Home Position Button
        self.home_button = Button(master, text="Set Home Position", command=self.set_home_position)
        self.home_button.pack(side="top", fill='both', expand=True, padx=10, pady=5)
        
        # Add go home Button (Clears waypoints and Adds Home)
        self.add_home_button = Button(master, text="Add Home", command=self.add_home)
        self.add_home_button.pack(side="top", fill='both', expand=True, padx=10, pady=5)

        # Add go home Button (Clears waypoints and Adds Home)
        self.go_home_button = Button(master, text="Go Home", command=self.go_home)
        self.go_home_button.pack(side="top", fill='both', expand=True, padx=10, pady=5)
        
        # Add Erase Waypoints Button
        self.erase_waypoints_button = Button(master, text="Erase Waypoints", command=self.erase_waypoints)
        self.erase_waypoints_button.pack(side="top", fill='both', expand=True, padx=10, pady=5)

        # Publish button
        self.publish_goal_waypoints = Button(master, text="Visualize Waypoint List", command=self.visualize_waypoints)
        self.publish_goal_waypoints.pack(side="top", fill='both', expand=True, padx=10, pady=5)

        # # Initialize ROS node
        self.pub1 = rospy.Publisher('new_pose_visualization', PoseStamped, queue_size=10)
        # self.pub2 = rospy.Publisher('waypoint_topic_2', Pose, queue_size=10)
        self.pub3 = rospy.Publisher("waypoint_plot_visualization", PoseArray, queue_size=10)

        # self.sub1 = rospy.Subscriber()

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

                ############
                # R1 = quaternion_matrix([self.last_waypoint.orientation.x, self.last_waypoint.orientation.y, self.last_waypoint.orientation.z, self.last_waypoint.orientation.w])
                # T1 = translation_matrix([self.last_waypoint.position.x, self.last_waypoint.position.y, self.last_waypoint.position.z])
        
                # # convert newest waypoint to global frame 
                # # R2 = euler_matrix(yaw, pitch,roll, 'rzyx') #sxyz
                # R2 = euler_matrix(roll,pitch,yaw) #, 'sxyz') 
                # T2 = translation_matrix([x,y,z])
    
                # transform1 = concatenate_matrices(R1,T1)
                # transform2 = concatenate_matrices(R2,T2)
                # transform = concatenate_matrices(transform1,transform2)
                # transform = concatenate_matrices(R2,T2,R1,T1)

                # # get values for glboal waypoint
                # translation = translation_from_matrix(transform)
                # quaternion = quaternion_from_matrix(transform)
                ##################


                # rospy.loginfo(f"need to convert this to global waypoint:\n {pose_msg}")
            
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'map_frame'
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
                            
            # formatting for rviz visualization
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.frame_id = 'map_frame'
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
