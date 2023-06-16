
class Config:
    SENSING_HORIZON = 8.0

    DEVICE = "/cpu:0"

    HOST_AGENT_OBSERVATION_LENGTH = 4   # dist to goal, heading to goal, pref speed, radius
    OTHER_AGENT_OBSERVATION_LENGTH = 7  # other px, other py, other vx, other vy, other radius, combined radius, distance between
    RNN_HELPER_LENGTH = 1               # num other agents
    AGENT_ID_LENGTH = 1

    # NN Input:
    # [num other agents, dist to goal, heading to goal, pref speed, radius,
    #  other px, other py, other vx, other vy, other radius, dist btwn, combined radius,
    #  other px, other py, other vx, other vy, other radius, dist btwn, combined radius,
    #  other px, other py, other vx, other vy, other radius, dist btwn, combined radius]
    MAX_NUM_OTHER_AGENTS_OBSERVED = 9   # depends on settings of saved network
    FULL_STATE_LENGTH = RNN_HELPER_LENGTH + HOST_AGENT_OBSERVATION_LENGTH + MAX_NUM_OTHER_AGENTS_OBSERVED * OTHER_AGENT_OBSERVATION_LENGTH
    FULL_LABELED_STATE_LENGTH = AGENT_ID_LENGTH + FULL_STATE_LENGTH
    FIRST_STATE_INDEX = 1
