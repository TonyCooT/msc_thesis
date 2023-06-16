
import numpy as np
import copy

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, Point
from nav_msgs.msg import Odometry
from social_msgs.msg import Peds

class Helper(Node):
    def __init__(self):
        super().__init__("rl_planner")

        self.declare_parameter("radius", 0.0)
        radius = self.get_parameter("radius").value

        # robot info
        self.robot_pose = PoseStamped()
        self.robot_position = np.array([0.0, 0.0])
        self.robot_velocity = np.array([0.0, 0.0])
        self.robot_time = 0.0

        # peds info
        self.num = 4
        self.peds_ids = np.array([1, 2, 3, 4, 5])
        self.peds_position = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
        self.peds_velocity = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
        self.peds_radius = radius
        self.peds_time = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # optional elements
        self.goal_pub_flag = True
        self.goal_pub_time = float(self.get_clock().now().nanoseconds / 10**9)
        self.radius_noise_flag = False
        self.robot_truth_position = np.array([0.0, 0.0])
        self.robot_truth_time = 0.0
        self.file_name = "output.txt"
        file = open(self.file_name,"w")
        file.close()

        # subscribers
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.sub_robot_position = self.create_subscription(Odometry, "/ground_truth", self.robot_truth_callback, qos_profile = qos_policy)
        self.sub_robot_position = self.create_subscription(Odometry, "/odometry/filtered", self.robot_callback, qos_profile = qos_policy)
        self.sub_ped_one_position = self.create_subscription(Odometry, "/crowd/ped_1", self.ped_one_callback, qos_profile = qos_policy)
        self.sub_ped_two_position = self.create_subscription(Odometry, "/crowd/ped_2", self.ped_two_callback, qos_profile = qos_policy)
        self.sub_ped_three_position = self.create_subscription(Odometry, "/crowd/ped_3", self.ped_three_callback, qos_profile = qos_policy)
        self.sub_ped_four_position = self.create_subscription(Odometry, "/crowd/ped_4", self.ped_four_callback, qos_profile = qos_policy)
        self.sub_ped_five_position = self.create_subscription(Odometry, "/crowd/ped_5", self.ped_five_callback, qos_profile = qos_policy)

        # publishers
        self.pub_robot_position = self.create_publisher(PoseStamped, "/rl_planner/pose", 1)
        self.pub_robot_velocity = self.create_publisher(Vector3, "/rl_planner/velocity", 1)
        self.pub_peds_info = self.create_publisher(Peds, "/rl_planner/peds", 1)
        self.pub_robot_goal = self.create_publisher(PoseStamped, "/rl_planner/goal", 1)

        # control timer
        self.robot_info_timer = self.create_timer(0.1, self.robot_info_callback)
        self.peds_info_timer = self.create_timer(0.1, self.peds_info_callback)
        self.output_timer = self.create_timer(0.25, self.output_callback)

    def robot_truth_callback(self, msg):
        self.robot_truth_position[0] = msg.pose.pose.position.x
        self.robot_truth_position[1] = msg.pose.pose.position.y
        self.robot_truth_time = float(self.get_clock().now().nanoseconds / 10**9)

    def robot_callback(self, msg):
        if self.robot_time == 0.0:
            self.robot_pose.pose = msg.pose.pose
            self.robot_position[0] = msg.pose.pose.position.x
            self.robot_position[1] = msg.pose.pose.position.y
            self.robot_time = float(self.get_clock().now().nanoseconds / 10**9)
            return

        self.robot_pose.pose = msg.pose.pose
        prev_robot_position = copy.deepcopy(self.robot_position)
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y

        now = float(self.get_clock().now().nanoseconds / 10**9)
        dt = now - self.robot_time
        self.robot_time = now

        self.robot_velocity = (self.robot_position - prev_robot_position) / dt

    def ped_one_callback(self, msg):
        if self.peds_time[0] == 0.0:
            self.peds_position[0,0] = msg.pose.pose.position.x
            self.peds_position[0,1] = msg.pose.pose.position.y
            self.peds_time[0] = float(self.get_clock().now().nanoseconds / 10**9)
            return

        prev_ped_position = copy.deepcopy(self.peds_position[0,:])
        self.peds_position[0,0] = msg.pose.pose.position.x
        self.peds_position[0,1] = msg.pose.pose.position.y

        now = float(self.get_clock().now().nanoseconds / 10**9)
        dt = now - self.peds_time[0]
        self.peds_time[0] = now

        self.peds_velocity[0,:] = (self.peds_position[0,:] - prev_ped_position) / dt

    def ped_two_callback(self, msg):
        if self.peds_time[1] == 0.0:
            self.peds_position[1,0] = msg.pose.pose.position.x
            self.peds_position[1,1] = msg.pose.pose.position.y
            self.peds_time[1] = float(self.get_clock().now().nanoseconds / 10**9)
            return

        prev_ped_position = copy.deepcopy(self.peds_position[1,:])
        self.peds_position[1,0] = msg.pose.pose.position.x
        self.peds_position[1,1] = msg.pose.pose.position.y

        now = float(self.get_clock().now().nanoseconds / 10**9)
        dt = now - self.peds_time[1]
        self.peds_time[1] = now

        self.peds_velocity[1,:] = (self.peds_position[1,:] - prev_ped_position) / dt

    def ped_three_callback(self, msg):
        if self.peds_time[2] == 0.0:
            self.peds_position[2,0] = msg.pose.pose.position.x
            self.peds_position[2,1] = msg.pose.pose.position.y
            self.peds_time[2] = float(self.get_clock().now().nanoseconds / 10**9)
            return

        prev_ped_position = copy.deepcopy(self.peds_position[2,:])
        self.peds_position[2,0] = msg.pose.pose.position.x
        self.peds_position[2,1] = msg.pose.pose.position.y

        now = float(self.get_clock().now().nanoseconds / 10**9)
        dt = now - self.peds_time[2]
        self.peds_time[2] = now

        self.peds_velocity[2,:] = (self.peds_position[2,:] - prev_ped_position) / dt

    def ped_four_callback(self, msg):
        if self.peds_time[3] == 0.0:
            self.peds_position[3,0] = msg.pose.pose.position.x
            self.peds_position[3,1] = msg.pose.pose.position.y
            self.peds_time[3] = float(self.get_clock().now().nanoseconds / 10**9)
            return

        prev_ped_position = copy.deepcopy(self.peds_position[3,:])
        self.peds_position[3,0] = msg.pose.pose.position.x
        self.peds_position[3,1] = msg.pose.pose.position.y

        now = float(self.get_clock().now().nanoseconds / 10**9)
        dt = now - self.peds_time[3]
        self.peds_time[3] = now

        self.peds_velocity[3,:] = (self.peds_position[3,:] - prev_ped_position) / dt

    def ped_five_callback(self, msg):
        if self.peds_time[4] == 0.0:
            self.peds_position[4,0] = msg.pose.pose.position.x
            self.peds_position[4,1] = msg.pose.pose.position.y
            self.peds_time[4] = float(self.get_clock().now().nanoseconds / 10**9)
            return

        prev_ped_position = copy.deepcopy(self.peds_position[4,:])
        self.peds_position[4,0] = msg.pose.pose.position.x
        self.peds_position[4,1] = msg.pose.pose.position.y

        now = float(self.get_clock().now().nanoseconds / 10**9)
        dt = now - self.peds_time[4]
        self.peds_time[4] = now

        self.peds_velocity[4,:] = (self.peds_position[4,:] - prev_ped_position) / dt

    def robot_info_callback(self):
        robot_pose_msg = PoseStamped()
        robot_pose_msg  = self.robot_pose

        robot_velocity_msg = Vector3()
        robot_velocity_msg.x = self.robot_velocity[0]
        robot_velocity_msg.y = self.robot_velocity[1]

        self.pub_robot_position.publish(robot_pose_msg)
        self.pub_robot_velocity.publish(robot_velocity_msg)

        now = float(self.get_clock().now().nanoseconds / 10**9)
        if self.once_flag and (now - self.goal_time) > 5.0:
            self.once_flag = False
            robot_goal_pose_msg = PoseStamped()
            robot_goal_pose_msg.pose.position.x = 10.0
            robot_goal_pose_msg.pose.position.y = -2.0
            self.pub_robot_goal.publish(robot_goal_pose_msg)

    def peds_info_callback(self):
        msg = Peds()
        for ped in range(self.num):
            msg.ids.append(self.peds_ids[ped])
            position = Point()
            position.x = self.peds_position[ped, 0]
            position.y = self.peds_position[ped, 1]
            msg.position.append(position)
            velocity = Vector3()
            velocity.x = self.peds_velocity[ped, 0]
            velocity.y = self.peds_velocity[ped, 1]
            msg.velocity.append(velocity)
            if self.radius_noise_flag:
                msg.radius.append(self.peds_radius + np.random.normal(0.0, 0.01, 1))
            else:
                msg.radius.append(self.peds_radius)
        self.pub_peds_info.publish(msg)

    def output_callback(self):
        with open(self.file_name, "a") as file:
            file.write(str(f"robot truth: {self.robot_truth_position[0]}, {self.robot_truth_position[1]}, {self.robot_truth_time}") + " \n")
            for ped in range(self.num):
                file.write(str(f"peds: {self.peds_ids[ped]}, {self.peds_position[ped,:]}") + " \n")

def main(args = None):
    rclpy.init(args = args)
    helper = Helper()
    try:
        rclpy.spin(helper)
    except KeyboardInterrupt:
        pass
    finally:
        helper.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
