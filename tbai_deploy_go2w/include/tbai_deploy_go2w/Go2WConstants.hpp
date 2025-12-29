#pragma once

namespace tbai {
namespace go2w {

// Go2W 16 DOF joint count (4 legs x 4 joints: hip, thigh, calf, wheel)
constexpr int GO2W_NUM_JOINTS = 16;

// Number of leg joints (position controlled)
constexpr int GO2W_NUM_LEG_JOINTS = 12;

// Number of wheel joints (velocity controlled)
constexpr int GO2W_NUM_WHEEL_JOINTS = 4;

// State vector size: 3 orientation + 3 position + 3 angular vel + 3 linear vel + 16 joint pos + 16 joint vel
constexpr int GO2W_STATE_DIM = 3 + 3 + 3 + 3 + GO2W_NUM_JOINTS + GO2W_NUM_JOINTS;

// Observation dimensions (per timestep, no history)
constexpr int OBS_ANG_VEL = 3;
constexpr int OBS_GRAVITY = 3;
constexpr int OBS_CMD = 3;
constexpr int OBS_DOF_ERR = GO2W_NUM_JOINTS;
constexpr int OBS_DOF_VEL = GO2W_NUM_JOINTS;
constexpr int OBS_DOF_POS = GO2W_NUM_JOINTS;
constexpr int OBS_LAST_ACTION = GO2W_NUM_JOINTS;

// Total observation size per step: 3 + 3 + 3 + 16 + 16 + 16 + 16 = 73
constexpr int OBS_SIZE_PER_STEP =
    OBS_ANG_VEL + OBS_GRAVITY + OBS_CMD + OBS_DOF_ERR + OBS_DOF_VEL + OBS_DOF_POS + OBS_LAST_ACTION;

// Observation history length for DreamWaQ encoder
constexpr int HISTORY_LENGTH = 5;

// Total observation history size: 73 * 5 = 365
constexpr int TOTAL_OBS_HISTORY_SIZE = OBS_SIZE_PER_STEP * HISTORY_LENGTH;

constexpr int LATENT_DIM = 16;
constexpr int VEL_DIM = 3;
constexpr int ENCODER_OUT_DIM = LATENT_DIM + VEL_DIM;  // 19

constexpr int ACTOR_INPUT_SIZE = ENCODER_OUT_DIM + OBS_SIZE_PER_STEP;

constexpr int WHEEL_SIM_INDICES[] = {3, 7, 11, 15};
constexpr int WHEEL_REAL_INDICES[] = {12, 13, 14, 15};

}  // namespace go2w
}  // namespace tbai
