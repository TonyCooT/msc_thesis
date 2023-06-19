
import numpy as np
import operator
from rl_planner.config import Config

class Agent():
    def __init__(self, start_x, start_y, goal_x, goal_y, radius, pref_speed, initial_heading, id):
        # Global Frame states
        self.pos_global_frame = np.array([start_x, start_y], dtype="float64")
        self.goal_global_frame = np.array([goal_x, goal_y], dtype="float64")
        self.vel_global_frame = np.array([pref_speed, 0.0], dtype="float64")
        self.speed_global_frame = 0.0
        self.heading_global_frame = initial_heading

        # Ego Frame states
        self.vel_ego_frame = np.array([0.0, 0.0])
        self.heading_ego_frame = 0.0

        # Other parameters
        self.id = id
        self.radius = radius
        self.pref_speed = pref_speed
        self.dist_to_goal = 0.0

        self.update_state()

    def update_state(self):
        # Compute heading w.r.t. ref_prll, ref_orthog coordinate axes
        self.ref_prll, self.ref_orth = self.get_ref()
        ref_prll_angle_global_frame = np.arctan2(self.ref_prll[1], self.ref_prll[0])
        self.heading_ego_frame = self.heading_global_frame - ref_prll_angle_global_frame
        if abs(self.heading_ego_frame) > np.pi:
            self.heading_ego_frame -= np.sign(self.heading_ego_frame) * 2 * np.pi

        # Compute velocity w.r.t. ref_prll, ref_orthog coordinate axes
        cur_speed = np.linalg.norm(self.vel_global_frame)
        v_prll = cur_speed * np.cos(self.heading_ego_frame)
        v_orthog = cur_speed * np.sin(self.heading_ego_frame)
        self.vel_ego_frame = np.array([v_prll, v_orthog])

    def observe(self, agents):
        obs = np.zeros((Config.FULL_LABELED_STATE_LENGTH))

        # Own agent state (ID is removed before inputting to NN, num other agents is used to rearrange other agents into sequence by NN)
        obs[0] = self.id
        obs[Config.AGENT_ID_LENGTH] = 0
        obs[Config.AGENT_ID_LENGTH + Config.FIRST_STATE_INDEX:Config.AGENT_ID_LENGTH + Config.FIRST_STATE_INDEX + Config.HOST_AGENT_OBSERVATION_LENGTH] = \
            self.dist_to_goal, self.heading_ego_frame, self.pref_speed, self.radius

        other_agent_dists = {}
        for i, other_agent in enumerate(agents):
            rel_pos_to_other_global_frame = other_agent.pos_global_frame - self.pos_global_frame
            dist_between_agent_centers = np.linalg.norm(rel_pos_to_other_global_frame)
            dist_to_other = dist_between_agent_centers - self.radius - other_agent.radius
            if dist_between_agent_centers > Config.SENSING_HORIZON:
                continue
            other_agent_dists[i] = dist_to_other

        # "closest_last" sorting method
        sorted_pairs = sorted(other_agent_dists.items(), key=operator.itemgetter(1))
        sorted_inds = [ind for (ind, pair) in sorted_pairs]
        sorted_inds.reverse()
        clipped_sorted_inds = sorted_inds[-Config.MAX_NUM_OTHER_AGENTS_OBSERVED:]
        clipped_sorted_agents = [agents[i] for i in clipped_sorted_inds]

        i = 0
        for other_agent in clipped_sorted_agents:
            # project other elements onto the new reference frame
            rel_pos_to_other_global_frame = other_agent.pos_global_frame - self.pos_global_frame
            p_parallel_ego_frame = np.dot(rel_pos_to_other_global_frame, self.ref_prll)
            p_orthog_ego_frame = np.dot(rel_pos_to_other_global_frame, self.ref_orth)
            v_parallel_ego_frame = np.dot(other_agent.vel_global_frame, self.ref_prll)
            v_orthog_ego_frame = np.dot(other_agent.vel_global_frame, self.ref_orth)
            dist_between_agent_centers = np.linalg.norm(rel_pos_to_other_global_frame)
            dist_to_other = dist_between_agent_centers - self.radius - other_agent.radius
            combined_radius = self.radius + other_agent.radius

            other_obs = np.array([p_parallel_ego_frame, p_orthog_ego_frame, v_parallel_ego_frame, v_orthog_ego_frame, other_agent.radius, combined_radius, dist_to_other])

            start_index = Config.AGENT_ID_LENGTH + Config.FIRST_STATE_INDEX + Config.HOST_AGENT_OBSERVATION_LENGTH + Config.OTHER_AGENT_OBSERVATION_LENGTH * i
            end_index = Config.AGENT_ID_LENGTH + Config.FIRST_STATE_INDEX + Config.HOST_AGENT_OBSERVATION_LENGTH + Config.OTHER_AGENT_OBSERVATION_LENGTH * (i + 1)

            obs[start_index:end_index] = other_obs
            i += 1

        obs[Config.AGENT_ID_LENGTH] = i # Will be used by RNN for seq_length

        return obs

    def get_ref(self):
        goal_direction = self.goal_global_frame - self.pos_global_frame
        self.dist_to_goal = np.linalg.norm(goal_direction)
        if self.dist_to_goal > 1e-8:
            ref_prll = goal_direction / self.dist_to_goal
        else:
            ref_prll = goal_direction
        ref_orth = np.array([-ref_prll[1], ref_prll[0]])
        return ref_prll, ref_orth
