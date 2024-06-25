import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import tf
import math

class SimRemap:
    def __init__(self):
        self.m=3
        # self.diff=[[0,0,0],[0,1,0],[0,-1,0]]#[x,y,delta]
        self.twist_pub = []
        self.odom_pub = []
        self.twist_sub = []
        # self.odom_sub = []
        
        for i in range(self.m):
            twist_pub = rospy.Publisher(f"/robot{i+1}/drive_controller/cmd_vel", Twist, queue_size=1000)
            odom_pub = rospy.Publisher(f"/robot{i+1}/Odometry", Odometry, queue_size=1000)
            self.twist_pub.append(twist_pub)
            self.odom_pub.append(odom_pub)

            twist_sub = rospy.Subscriber(f"/robot{i+1}/cmd_vel", Twist, self.make_twist_callback(i))
            self.twist_sub.append(twist_sub)
        self.odom_sub = rospy.Subscriber(f"/gazebo/model_states", ModelStates, self.odom_callback)
        rospy.loginfo("Initialize complete")
    
    def make_twist_callback(self, i):
        def callback(msg:Twist):
            rospy.logdebug(f"relay twist {i+1}")
            self.twist_pub[i].publish(msg)
        return callback
    
    def odom_callback(self,msg:ModelStates):
        for i in range(self.m):
            robot_name = f"robot{i + 1}"
            if robot_name in msg.name:
                index = msg.name.index(robot_name)
                odom_msg = Odometry()
                odom_msg.header.stamp = rospy.Time.now()
                odom_msg.header.frame_id = f"/robot{i + 1}/odom"
                odom_msg.child_frame_id = f"/robot{i + 1}/base_link"

                # Pose transformation: y direction to x direction
                pose = msg.pose[index]
                position = pose.position

                # Rotate position
                new_position_x = position.y
                new_position_y = -position.x
                new_position_z = position.z

                odom_msg.pose.pose.position.x = new_position_x
                odom_msg.pose.pose.position.y = new_position_y
                odom_msg.pose.pose.position.z = new_position_z

                odom_msg.pose.pose.orientation = pose.orientation

                # Twist transformation: y direction to x direction
                twist = msg.twist[index]
                new_linear_x = twist.linear.y
                new_linear_y = -twist.linear.x
                new_linear_z = twist.linear.z

                new_angular_x = twist.angular.y
                new_angular_y = -twist.angular.x
                new_angular_z = twist.angular.z

                odom_msg.twist.twist.linear.x = new_linear_x
                odom_msg.twist.twist.linear.y = new_linear_y
                odom_msg.twist.twist.linear.z = new_linear_z

                odom_msg.twist.twist.angular.x = new_angular_x
                odom_msg.twist.twist.angular.y = new_angular_y
                odom_msg.twist.twist.angular.z = new_angular_z

                rospy.logdebug(f"relay odom {i + 1}")
                self.odom_pub[i].publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('sim_remap', anonymous=True)
    sim_remap = SimRemap()
    rospy.spin()
