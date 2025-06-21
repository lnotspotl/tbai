import time

import sys

sys.path.insert(0, "/home/kuba/Documents/tbai2/build/tbai_python")
import numpy as np

from collections import OrderedDict

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

import mpac_logging
from mpac_logging.rerun.robot_logger import RobotLogger
from mpac_logging.rerun.utils import rerun_initialize, rerun_store

from tbai_python.rotations import rpy2quat, quat2mat, mat2rpy, mat2ocs2rpy, ocs2rpy2quat, rpy2mat, mat2aa

LegID = {
    "FR_0": 0,  # Front right hip
    "FR_1": 1,  # Front right thigh
    "FR_2": 2,  # Front right calf
    "FL_0": 3,
    "FL_1": 4,
    "FL_2": 5,
    "RR_0": 6,
    "RR_1": 7,
    "RR_2": 8,
    "RL_0": 9,
    "RL_1": 10,
    "RL_2": 11,
}

HIGHLEVEL = 0xEE
LOWLEVEL = 0xFF
TRIGERLEVEL = 0xF0
PosStopF = 2.146e9
VelStopF = 16000.0


class Custom:
    def __init__(self):
        self.Kp = 60
        self.Kd = 0.5
        self.time_consume = 0
        self.rate_count = 0
        self.sin_count = 0
        self.motiontime = 0
        self.dt = 0.002  # 0.001~0.01

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None

        self._targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65, -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        self._targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        self._targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65, -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]

        self.startPos = [0.0] * 12
        self.duration_1 = 500
        self.duration_2 = 500
        self.duration_3 = 1000
        self.duration_4 = 900
        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0

        self.firstRun = True
        self.done = False

        # thread handling
        self.lowCmdWriteThreadPtr = None

        rerun_initialize("go2_logger", spawn=False)
        self.rerun_logger = RobotLogger.from_zoo("go2_description")

        self.log_freq = 10
        self.last_log_time = None
        self.last_yaw = 0.0

        self.crc = CRC()

    # Public methods
    def Init(self):
        self.InitLowCmd()

        # create publisher #
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # create subscriber #
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.sc = SportClient()
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result["name"]:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(interval=1/50, target=self.LowCmdWrite, name="writebasiccmd")
        self.lowCmdWriteThreadPtr.Start()

    # Private methods
    def InitLowCmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q = PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

        if self.last_log_time is None:
            self.last_log_time = time.time()

        if time.time() - self.last_log_time < 1 / self.log_freq:
            return
        self.last_log_time = time.time()

        position = np.array([0.0, 0.0, 0.27])
        orientation_xyz = np.roll(np.array(self.low_state.imu_state.quaternion), -1)
        joint_positions = OrderedDict()
        joint_positions["FL_hip_joint"] = self.low_state.motor_state[LegID["FL_0"]].q
        joint_positions["FL_thigh_joint"] = self.low_state.motor_state[LegID["FL_1"]].q
        joint_positions["FL_calf_joint"] = self.low_state.motor_state[LegID["FL_2"]].q
        joint_positions["RL_hip_joint"] = self.low_state.motor_state[LegID["RL_0"]].q
        joint_positions["RL_thigh_joint"] = self.low_state.motor_state[LegID["RL_1"]].q
        joint_positions["RL_calf_joint"] = self.low_state.motor_state[LegID["RL_2"]].q
        joint_positions["FR_hip_joint"] = self.low_state.motor_state[LegID["FR_0"]].q
        joint_positions["FR_thigh_joint"] = self.low_state.motor_state[LegID["FR_1"]].q
        joint_positions["FR_calf_joint"] = self.low_state.motor_state[LegID["FR_2"]].q
        joint_positions["RR_hip_joint"] = self.low_state.motor_state[LegID["RR_0"]].q
        joint_positions["RR_thigh_joint"] = self.low_state.motor_state[LegID["RR_1"]].q
        joint_positions["RR_calf_joint"] = self.low_state.motor_state[LegID["RR_2"]].q

        joint_velocities = OrderedDict()
        joint_velocities["FL_hip_joint"] = self.low_state.motor_state[LegID["FL_0"]].dq
        joint_velocities["FL_thigh_joint"] = self.low_state.motor_state[LegID["FL_1"]].dq
        joint_velocities["FL_calf_joint"] = self.low_state.motor_state[LegID["FL_2"]].dq
        joint_velocities["RL_hip_joint"] = self.low_state.motor_state[LegID["RL_0"]].dq
        joint_velocities["RL_thigh_joint"] = self.low_state.motor_state[LegID["RL_1"]].dq
        joint_velocities["RL_calf_joint"] = self.low_state.motor_state[LegID["RL_2"]].dq
        joint_velocities["FR_hip_joint"] = self.low_state.motor_state[LegID["FR_0"]].dq
        joint_velocities["FR_thigh_joint"] = self.low_state.motor_state[LegID["FR_1"]].dq
        joint_velocities["FR_calf_joint"] = self.low_state.motor_state[LegID["FR_2"]].dq
        joint_velocities["RR_hip_joint"] = self.low_state.motor_state[LegID["RR_0"]].dq
        joint_velocities["RR_thigh_joint"] = self.low_state.motor_state[LegID["RR_1"]].dq
        joint_velocities["RR_calf_joint"] = self.low_state.motor_state[LegID["RR_2"]].dq

        base_position = np.array([0.0, 0.0, 0.27])  # this should not be necessary
        R_world_base = quat2mat(orientation_xyz)
        rpy = mat2ocs2rpy(R_world_base, self.last_yaw)
        self.last_yaw = rpy[2]

        angular_velocity_base = self.low_state.imu_state.gyroscope
        linear_velocity_base = np.array([0.0, 0.0, 0.0])  # this should not be necessary

        joint_positions_ocs2 = np.array(list(joint_positions.values()))
        joint_velocities_ocs2 = np.array(list(joint_velocities.values()))

        ocs2state = np.concatenate(
            (rpy, base_position, angular_velocity_base, linear_velocity_base, joint_positions_ocs2, joint_velocities_ocs2)
        )

        position_ocs2 = ocs2state[3:6]
        orientation_ocs2 = ocs2state[0:3]
        joint_positions_ocs2 = ocs2state[12:24]

        joint_positions_rerun = {
            "FL_hip_joint": joint_positions_ocs2[0],
            "FL_thigh_joint": joint_positions_ocs2[1],
            "FL_calf_joint": joint_positions_ocs2[2],
            "RL_hip_joint": joint_positions_ocs2[3],
            "RL_thigh_joint": joint_positions_ocs2[4],
            "RL_calf_joint": joint_positions_ocs2[5],
            "FR_hip_joint": joint_positions_ocs2[6],
            "FR_thigh_joint": joint_positions_ocs2[7],
            "FR_calf_joint": joint_positions_ocs2[8],
            "RR_hip_joint": joint_positions_ocs2[9],
            "RR_thigh_joint": joint_positions_ocs2[10],
            "RR_calf_joint": joint_positions_ocs2[11],
        }

        # Turn orientation
        orientation_ocs2 = ocs2rpy2quat(orientation_ocs2)

        # Log the new state
        self.rerun_logger.log_state(
            logtime=time.time(),
            base_position=position_ocs2,
            base_orientation=orientation_ocs2,
            joint_positions=joint_positions_rerun,
        )

        self.rerun_logger.log_state(
            logtime=time.time(),
            base_position=position,
            base_orientation=orientation_xyz,
            joint_positions=joint_positions,
        )

        # print("FR_0 motor state: ", msg.motor_state[LegID["FR_0"]])
        # print("IMU state: ", msg.imu_state)
        # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)

    def LowCmdWrite(self):
        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False

        for i in range(12):
            self.low_cmd.motor_cmd[i].q = self.low_state.motor_state[i].q
            self.low_cmd.motor_cmd[i].dq = self.low_state.motor_state[i].dq
            self.low_cmd.motor_cmd[i].kp = self.Kp * 0
            self.low_cmd.motor_cmd[i].kd = self.Kd * 0
            self.low_cmd.motor_cmd[i].tau = 0

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


if __name__ == "__main__":
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(0)

    custom = Custom()
    custom.Init()
    custom.Start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
        rerun_store("go2_logger.rrd")
