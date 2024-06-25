import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class SimRemap:
    def __init__(self):
        self.m=3
        self.diff=[[0,0,0],[1,0,0],[-1,0,0]]#[x,y,delta]
        self.twist_pub = []
        self.odom_pub = []
        self.twist_sub = []
        self.odom_sub = []
        
        for i in range(self.m):
            twist_pub = rospy.Publisher(f"/robot{i+1}/drive_controller/cmd_vel", Twist, queue_size=1000)
            odom_pub = rospy.Publisher(f"/robot{i+1}/Odometry", Odometry, queue_size=1000)
            self.twist_pub.append(twist_pub)
            self.odom_pub.append(odom_pub)

            twist_sub = rospy.Subscriber(f"/robot{i+1}/cmd_vel", Twist, self.make_twist_callback(i))
            odom_sub = rospy.Subscriber(f"/robot{i+1}/drive_controller/odom", Odometry, self.make_odom_callback(i))
            self.twist_sub.append(twist_sub)
            self.odom_sub.append(odom_sub)
        rospy.loginfo("Initialize complete")
    
    def make_twist_callback(self, i):
        def callback(msg:Twist):
            rospy.logdebug(f"relay twist {i+1}")
            self.twist_pub[i].publish(msg)
        return callback
    
    def make_odom_callback(self, i):
        def callback(msg:Odometry):
            rospy.logdebug(f"relay odom {i+1}")
            msg.pose.pose.position.x+=self.diff[i][0]
            msg.pose.pose.position.y+=self.diff[i][1]
            self.odom_pub[i].publish(msg)
        return callback

if __name__ == '__main__':
    rospy.init_node('sim_remap', anonymous=True)
    sim_remap = SimRemap()
    rospy.spin()
