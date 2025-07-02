#!/usr/bin/env python3

import sys

sys.path.insert(0, "/home/kuba/Documents/tbai2/build/tbai_python")

import tbai_python
import numpy as np
import time
import threading

from collections import OrderedDict

from joystick import UIController


from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_

from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

import mpac_logging
from mpac_logging.rerun.robot_logger import RobotLogger
from mpac_logging.rerun.utils import rerun_initialize, rerun_store


from tbai_python import (
    StateSubscriber,
    CommandPublisher,
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

from tbai_python.rotations import rpy2quat, quat2mat, mat2rpy, mat2ocs2rpy, ocs2rpy2quat, rpy2mat, mat2aa

joint2idx = {
    "RF_HAA": 0,
    "RF_HFE": 1,
    "RF_KFE": 2,
    "LF_HAA": 3,
    "LF_HFE": 4,
    "LF_KFE": 5,
    "RH_HAA": 6,
    "RH_HFE": 7,
    "RH_KFE": 8,
    "LH_HAA": 9,
    "LH_HFE": 10,
    "LH_KFE": 11,
}

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


class Go2StateSubscriber(StateSubscriber):
    def __init__(self):
        super().__init__()

        self.current_state = None

        # create subscriber #
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.low_level_state_callback, 10)

        self.state_update_freq = 100
        self.last_update_time = None

        self.lock = threading.Lock()
        self.last_yaw = 0.0

    def low_level_state_callback(self, msg: LowState_):
        self.low_state = msg

        if self.last_update_time is None:
            self.last_update_time = time.time()

        if time.time() - self.last_update_time < 1 / self.state_update_freq:
            return
        self.last_update_time = time.time()

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
            (
                rpy,
                base_position,
                angular_velocity_base,
                linear_velocity_base,
                joint_positions_ocs2,
                joint_velocities_ocs2,
            )
        )

        assert len(ocs2state) == 36

        with self.lock:
            self.current_state = ocs2state
            self.current_state_timestamp = time.time()

    def waitTillInitialized(self):
        while self.current_state is None:
            print("Waiting for state")
            time.sleep(0.1)
        print("State subscriber initialized")

    def getLatestRbdState(self):
        with self.lock:
            return self.current_state

    def getLatestRbdStamp(self):
        with self.lock:
            return self.current_state_timestamp

    def getContactFlags(self):
        return [False] * 4


class RerunLoggerNode:
    def __init__(self, state_subscriber: StateSubscriber, freq=10):
        rerun_initialize("go2_deploy", spawn=False)
        self.robot_logger = RobotLogger.from_zoo("go2_description")
        self.state_subscriber = state_subscriber
        self.freq = freq
        self.last_time = None

    def visualize_callback(self, current_time, dt):
        if self.last_time is None:
            self.last_time = current_time

        if current_time - self.last_time < 1 / self.freq:
            return
        self.last_time = current_time
        state = self.state_subscriber.getLatestRbdState()
        position = state[3:6]
        orientation = state[0:3]
        joint_positions = state[12:24]

        joint_positions = {
            "FL_hip_joint": joint_positions[0],
            "FL_thigh_joint": joint_positions[1],
            "FL_calf_joint": joint_positions[2],
            "RL_hip_joint": joint_positions[3],
            "RL_thigh_joint": joint_positions[4],
            "RL_calf_joint": joint_positions[5],
            "FR_hip_joint": joint_positions[6],
            "FR_thigh_joint": joint_positions[7],
            "FR_calf_joint": joint_positions[8],
            "RR_hip_joint": joint_positions[9],
            "RR_thigh_joint": joint_positions[10],
            "RR_calf_joint": joint_positions[11],
        }

        # Turn orientation
        orientation = ocs2rpy2quat(orientation)

        # Log the new state
        self.robot_logger.log_state(
            logtime=current_time, base_position=position, base_orientation=orientation, joint_positions=joint_positions
        )


class Go2CommandPublisher(CommandPublisher):
    def __init__(self):
        super().__init__()

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.crc = CRC()

        self.last_time = time.time()
        self.publish_count = 0
        self.start_time = time.time()

        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            PosStopF = 2.146e9
            VelStopF = 16000.0
            self.low_cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
            self.low_cmd.motor_cmd[i].q = PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()


        # Running the commands bellow is necessary as the robot's motors otherwise rumble
        self.sc = SportClient()  
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        status, result = self.msc.CheckMode()
        while result['name']:
            self.sc.StandDown()
            self.msc.ReleaseMode()
            status, result = self.msc.CheckMode()
            time.sleep(1)

    def publish(self, commands):
        for command in commands:
            motor_index = joint2idx[command.joint_name]
            self.low_cmd.motor_cmd[motor_index].q = command.desired_position
            self.low_cmd.motor_cmd[motor_index].dq = command.desired_velocity
            self.low_cmd.motor_cmd[motor_index].kp = command.kp
            self.low_cmd.motor_cmd[motor_index].kd = command.kd
            self.low_cmd.motor_cmd[motor_index].tau = command.torque_ff

        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)


class Go2ChangeControllerSubscriber(ChangeControllerSubscriber):
    def __init__(self):
        super().__init__()
        self._callback = None
        self.new_controller = None
        self.new_controller = "SIT"

    def setCallbackFunction(self, callback):
        self._callback = callback

    def triggerCallbacks(self):
        if self._callback is not None and self.new_controller is not None:
            self._callback(self.new_controller)
            self.new_controller = None

    def stand_callback(self):
        self.new_controller = "STAND"

    def sit_callback(self):
        self.new_controller = "SIT"

    def bob_callback(self):
        self.new_controller = "BOB"


class Go2ReferenceVelocityGenerator(ReferenceVelocityGenerator):
    def __init__(self, ui_controller: UIController):
        super().__init__()
        self.ui_controller = ui_controller

    def getReferenceVelocity(self, time, dt):
        ref = ReferenceVelocity()
        ref.velocity_x = self.ui_controller.linear_x
        ref.velocity_y = self.ui_controller.linear_y
        ref.yaw_rate = self.ui_controller.angular_z
        return ref
    
assert len(sys.argv) == 2, "Usage: python deploy.py net-id"
ChannelFactoryInitialize(0, sys.argv[1])


# Initialize tbai components
subscriber = Go2StateSubscriber()
publisher = Go2CommandPublisher()
controller_sub = Go2ChangeControllerSubscriber()

rerun_logger = RerunLoggerNode(subscriber)

ui_controller = UIController(
    stand_callback=controller_sub.stand_callback,
    sit_callback=controller_sub.sit_callback,
    bob_callback=controller_sub.bob_callback,
)
ref_vel_gen = Go2ReferenceVelocityGenerator(ui_controller)

tbai_python.write_init_time()

central_controller = tbai_python.CentralController.create(subscriber, publisher, controller_sub)
central_controller.add_bob_controller(subscriber, ref_vel_gen, rerun_logger.visualize_callback)
central_controller.add_static_controller(subscriber, rerun_logger.visualize_callback)

central_controller.startThread()

try:
    ui_controller.run()
except KeyboardInterrupt:
    print("Stopping threads")
    central_controller.stopThread()
    rerun_store("go2.rrd")
