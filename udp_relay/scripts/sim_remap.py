import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import math,tf
import tf.transformations

use_angular_PID = False
use_self_angle_ref = False

class PIDController:
    def __init__(self, kp, ki, kd, i_clamp, min_output, max_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_clamp = i_clamp
        self.min_output = min_output
        self.max_output = max_output
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time=rospy.Time.now()

    def update(self, error):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.integral += error * dt
        self.integral = max(min(self.integral, self.i_clamp), -self.i_clamp)

        try:
            derivative = (error - self.last_error) / dt
        except ZeroDivisionError:
            derivative=0
        self.last_error = error

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), self.min_output)
        self.last_time = current_time
        return output

class SimRemap:
    def __init__(self):
        self.m=3
        # self.diff=[[0,0,0],[0,1,0],[0,-1,0]]#[x,y,delta]
        self.twist_pub = []
        self.odom_pub = []
        self.twist_sub = []
        
        self.last_yaw = {}
        # self.odom_sub = []
        # self.pid_linear = PIDController(kp=1.0, ki=0.2, kd=0.1, i_clamp=1, min_output=-1, max_output=1)
        self.pid_angular = PIDController(kp=2, ki=0, kd=0.1, i_clamp=1, min_output=-1, max_output=1)
        
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
            if self.current_yaw is None or i not in self.current_yaw:
                self.twist_pub[i].publish(msg)
            else:
                # linear_output = self.pid_linear.update(msg.linear.x - self.current_linear_x[i])+msg.linear.x
                if abs(msg.angular.z)>1e-6 or (i not in self.last_yaw):
                    self.last_yaw[i]=self.current_yaw[i]
                if use_self_angle_ref:
                    msg.linear.z=self.last_yaw[i]
                diff=msg.linear.z - self.current_yaw[i]
                while diff>math.pi:
                    diff-=math.pi
                while diff<-math.pi:
                    diff+=math.pi
                angular_output = self.pid_angular.update(diff)+msg.angular.z
                # angular_output = msg.angular.z
                print(f"diff = {diff:.3f}, ang v = {msg.angular.z:.3f}, ang ctrl = {angular_output:.3f}")
                
                new_cmd = Twist()
                # 不使用速度PID控制
                # new_cmd.linear.x = linear_output
                new_cmd.linear.x = msg.linear.x
                if use_angular_PID:
                    new_cmd.angular.z = angular_output
                else:
                    new_cmd.angular.z = msg.angular.z
                self.twist_pub[i].publish(new_cmd)
            rospy.logdebug(f"relay twist {i+1}")
        return callback
    
    def odom_callback(self,msg:ModelStates):
        self.current_linear_x = {}
        self.current_angular_z = {}
        self.current_yaw = {}
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
                odom_msg.pose.pose = pose

                # Twist transformation: y direction to x direction
                twist = msg.twist[index]
                odom_msg.twist.twist = twist

                self.current_linear_x[i] = twist.linear.x
                self.current_angular_z[i] = twist.angular.z
                rpy=tf.transformations.euler_from_quaternion([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w])
                self.current_yaw[i] = rpy[2]

                rospy.logdebug(f"relay odom {i + 1}")
                self.odom_pub[i].publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('sim_remap', anonymous=True)
    sim_remap = SimRemap()
    rospy.spin()
