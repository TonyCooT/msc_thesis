
import os
import numpy as np
import copy

from ament_index_python.packages import get_package_share_path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, PoseStamped, Twist
from social_msgs.msg import Peds

from rl_planner.agent import Agent
from rl_planner.config import Config
from rl_planner.network import Actions, NetworkVPRNN

class RlPlanner(Node):
    def __init__(self):
        super().__init__("rl_planner")

        self.declare_parameter("checkpoint", "network_00000000")
        self.declare_parameter("pref_speed", 0.0)
        self.declare_parameter("radius", 0.0)
        self.declare_parameter("near_goal_threshold", 0.0)
        self.declare_parameter("sensing_horizon", 0.0)
        self.declare_parameter("max_num_other_agent_observed", 0)

        checkpoint = self.get_parameter("checkpoint").value
        pref_speed = self.get_parameter("pref_speed").value
        radius = self.get_parameter("radius").value
        near_goal_threshold = self.get_parameter("near_goal_threshold").value
        sensing_horizon = self.get_parameter("sensing_horizon").value
        max_num_other_agent_observed = self.get_parameter("max_num_other_agent_observed").value

        Config.SENSING_HORIZON = sensing_horizon
        Config.MAX_NUM_OTHER_AGENTS_OBSERVED = max_num_other_agent_observed

        # robot info
        self.pref_speed = pref_speed
        self.radius = radius
        self.desired_action = np.zeros(2)
        self.near_goal_threshold = near_goal_threshold

        actions = Actions()
        nn = NetworkVPRNN(Config.DEVICE)
        pkg_share = os.path.join(get_package_share_path("rl_planner"))
        nn.simple_load(os.path.join(pkg_share, "checkpoints", checkpoint))

        # neural network
        self.nn = nn
        self.actions = actions.actions
        self.operation_mode = 1 # 1 - NN mode, 2 - Spin mode

        # for subscribers
        self.pose = PoseStamped()
        self.psi = 0.0
        self.vel = Vector3()
        self.global_goal = PoseStamped()
        self.other_agents_state = []

        # additional flags
        self.stop_moving_flag = True
        self.new_global_goal_received = False

        # publishers and subscribers
        self.pub_twist = self.create_publisher(Twist, "cmd_vel", 1)
        self.sub_pose = self.create_subscription(PoseStamped, "pose", self.pose_callback, 1)
        self.sub_vel = self.create_subscription(Vector3, "velocity", self.velocity_callback, 1)
        self.sub_peds = self.create_subscription(Peds, "peds", self.peds_callback, 1)
        self.sub_global_goal = self.create_subscription(PoseStamped, "goal", self.goal_callback, 1)

        # control timer
        self.control_timer = self.create_timer(0.01, self.control_callback)
        self.nn_timer = self.create_timer(0.1, self.compute_action_callback)

        self.get_logger().info("Done")

    def pose_callback(self, msg):
        self.pose = msg
        q = msg.pose.orientation
        self.psi = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def velocity_callback(self, msg):
        self.vel = msg

    def goal_callback(self, msg):
        self.global_goal = msg
        self.operation_mode = 2
        self.new_global_goal_received = True

    def peds_callback(self, msg):
        other_agents = []
        num_peds = len(msg.ids)

        for i in range(num_peds):
            index = msg.ids[i]
            x = msg.position[i].x
            y = msg.position[i].y
            v_x = msg.velocity[i].x
            v_y = msg.velocity[i].y
            radius = msg.radius[i]

            pref_speed = np.linalg.norm(np.array([v_x, v_y]))
            if pref_speed < 0.1:
                pref_speed = 0
            heading_angle = np.arctan2(v_y, v_x)
            goal_x = x + 5.0
            goal_y = y + 5.0

            other_agent = Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, index)
            other_agent.vel_global_frame = np.array([v_x, v_y])
            other_agents.append(other_agent)

        self.other_agents_state = other_agents

    def control_callback(self):
        if self.stop_moving_flag and not self.new_global_goal_received:
            self.stop_moving()
            return

        if self.operation_mode == 1:
            desired_yaw = self.desired_action[1]
            yaw_error = self.wrap(desired_yaw - self.psi)

            vx = self.desired_action[0]
            gain = 4.0
            vw = gain * yaw_error

            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = vw
            self.pub_twist.publish(msg)
        elif self.operation_mode == 2:
            self.stop_moving_flag = False
            angle_to_goal = np.arctan2(self.global_goal.pose.position.y - self.pose.pose.position.y, self.global_goal.pose.position.x - self.pose.pose.position.x)
            global_yaw_error = angle_to_goal - self.psi
            if abs(global_yaw_error) > 0.5:
                vx = 0.0
                vw = np.sign(global_yaw_error) * 1.0
                msg = Twist()
                msg.linear.x = vx
                msg.angular.z = vw
                self.pub_twist.publish(msg)
            else:
                self.operation_mode = 1
                self.new_global_goal_received = False
        else:
            self.stop_moving()

    def compute_action_callback(self):
        if self.operation_mode != 1 or self.stop_moving_flag:
            return

        # current state of host agent
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        v_x = self.vel.x
        v_y = self.vel.y
        radius = self.radius
        pref_speed = self.pref_speed
        heading_angle = self.psi
        goal_x = self.global_goal.pose.position.x
        goal_y = self.global_goal.pose.position.y

        # in case current speed is larger than desired speed
        v = np.linalg.norm(np.array([v_x, v_y]))
        if v > pref_speed:
            v_x = v_x * pref_speed / v
            v_y = v_y * pref_speed / v

        host_agent = Agent(x, y, goal_x, goal_y, radius, pref_speed, heading_angle, 0)
        host_agent.vel_global_frame = np.array([v_x, v_y])

        # convert agent states into observation vector and query the policy
        other_agents_state = copy.deepcopy(self.other_agents_state)
        obs = host_agent.observe(other_agents_state)[1:]
        #self.get_logger().info(f"[observe]: {obs}')
        obs = np.expand_dims(obs, axis=0)
        predictions = self.nn.predict_p(obs)[0]

        raw_action = copy.deepcopy(self.actions[np.argmax(predictions)])
        action = np.array([pref_speed * raw_action[0], self.wrap(raw_action[1] + self.psi)])

        if host_agent.dist_to_goal < self.near_goal_threshold:
            self.stop_moving_flag = True

        self.desired_action = action

    def stop_moving(self):
        msg = Twist()
        self.pub_twist.publish(msg)

    def on_shutdown(self):
        self.stop_moving()

    def wrap(self, angle):
        if abs(angle) > np.pi:
            angle -= np.sign(angle) * 2 * np.pi
        return angle

def main(args = None):
    rclpy.init(args = args)
    rl_planner = RlPlanner()
    try:
        rclpy.spin(rl_planner)
    except KeyboardInterrupt:
        pass
    finally:
        rl_planner.on_shutdown()
        rl_planner.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
