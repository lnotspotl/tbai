import sys
import copy

sys.path.insert(0, "/home/kuba/Documents/tbai2/build/tbai_python")

import tbai_python

import mpac_logging
from mpac_logging.rerun.robot_logger import RobotLogger
from mpac_logging.rerun.utils import rerun_initialize, rerun_store

import mujoco
import numpy as np
import time
import threading

from mujoco import viewer

import tbai_python
import numpy as np

from joystick import UIController

import time

from tbai_python import (
    StateSubscriber,
    CommandPublisher,
    ChangeControllerSubscriber,
    ReferenceVelocity,
    ReferenceVelocityGenerator,
)

from tbai_python.rotations import rpy2quat, quat2mat, mat2rpy, mat2ocs2rpy, ocs2rpy2quat, rpy2mat, mat2aa

kp = 30
kd = 0.5
desired_joint_angles = np.zeros(12)
running = True

joint2idx = {
    "LF_HAA": 0,
    "LF_HFE": 1,
    "LF_KFE": 2,
    "RF_HAA": 3,
    "RF_HFE": 4,
    "RF_KFE": 5,
    "LH_HAA": 6,
    "LH_HFE": 7,
    "LH_KFE": 8,
    "RH_HAA": 9,
    "RH_HFE": 10,
    "RH_KFE": 11,
}


def pd_controller(q_current, q_desired, v_desired, v_current, kp, kd) -> np.ndarray:
    return kp * (q_desired - q_current) + kd * (v_desired - v_current)


def viewer_fn(window, lock, viewer_dt):
    while window.is_running() and running:
        with lock:
            window.sync()
        time.sleep(viewer_dt)


def physics_fn(model, data, lock, physics_dt, subscriber):
    global desired_joint_angles
    while running:
        with lock:
            current_q = data.qpos[7 : 7 + 12]
            desired_q = desired_joint_angles.copy()
            current_v = data.qvel[6 : 6 + 12]
            desired_v = np.zeros_like(current_v)
            data.ctrl[:12] = pd_controller(current_q, desired_q, desired_v, current_v, kp, kd)
            mujoco.mj_step(model, data)
            subscriber.needs_update = True
        time.sleep(physics_dt)


class RerunLoggerNode:
    def __init__(self, state_subscriber: StateSubscriber, freq=10):
        rerun_initialize("simple_robot_example", spawn=False)
        self.robot_logger = RobotLogger.from_zoo("go2_description")
        self.state_subscriber = state_subscriber
        self.freq = freq
        self.last_time = None

    def visualize_callback(self, current_time, dt):
        if self.last_time is None:
            self.last_time = current_time

        if current_time - self.last_time < 1 / self.freq:
            return

        current_state = self.state_subscriber.getLatestState()
        self.last_time = current_time
        position = current_state.x[3:6]
        orientation = current_state.x[0:3]
        joint_positions = current_state.x[12:24]

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

        logtime = current_state.timestamp

        # Log the new state
        self.robot_logger.log_state(
            logtime=logtime, base_position=position, base_orientation=orientation, joint_positions=joint_positions
        )


class DummyStateSubscriber(StateSubscriber):
    def __init__(self, model, data):
        super().__init__()
        self.initialized = False
        self.current_state = np.zeros((36,), dtype=np.float64)
        self.model = model
        self.data = data
        self.last_time = time.time()
        self.last_orientation = np.eye(3)
        self.last_position = np.zeros(3)
        self.last_joint_angles = np.zeros(12)
        self.last_yaw = 0.0
        self.first_update = True
        self.needs_update = True
        self.ekf = tbai_python.TbaiEstimator(["FL_foot", "FR_foot", "RL_foot", "RR_foot"])
        self.last_velocity_base = None

    def updateStateThread(self):
        while running:
            self.getLatestRbdState()
            time.sleep(0.0025)

    def waitTillInitialized(self):
        self.initialized = True

    def getLatestState(self):
        return self.current_state

    def getLatestRbdState(self):
        if not self.needs_update:
            return self.current_state
        self.needs_update = False
        current_time = time.time()
        dt = current_time - self.last_time

        # Get base orientation (quat) and convert to rotation matrix
        base_quat = np.roll(self.data.qpos[3:7], -1)  # roll to make it xyzw
        R_world_base = quat2mat(base_quat)
        R_base_world = R_world_base.T

        # Get RPY angles
        rpy = mat2ocs2rpy(R_world_base, self.last_yaw)
        self.last_yaw = rpy[2]

        # Base position in world frame
        base_position = self.data.qpos[0:3]

        if self.first_update:
            self.last_orientation = R_base_world
            self.last_position = base_position
            self.first_update = False

        # Base angular velocity in base frame
        angular_velocity_world = mat2aa(R_world_base @ self.last_orientation) / dt if dt > 0 else np.zeros(3)
        angular_velocity_base = R_base_world @ angular_velocity_world

        # Base linear velocity in base frame
        linear_velocity_world = (base_position - self.last_position) / dt if dt > 0 else np.zeros(3)
        linear_velocity_base = R_base_world @ linear_velocity_world

        # Joint angles and velocities
        joint_angles = self.data.qpos[7:19].copy()  # Assuming 12 joints starting at index 7
        joint_angles[3:6], joint_angles[6:9] = joint_angles[3:6], joint_angles[6:9]
        joint_velocities = self.data.qvel[6:18].copy()  # Assuming 12 joint velocities starting at index 6
        joint_velocities[3:6], joint_velocities[6:9] = joint_velocities[3:6], joint_velocities[6:9]

        # Update state vector
        self.current_state[0:3] = rpy.copy()  # Base orientation (RPY)
        self.current_state[3:6] = base_position.copy()  # Base position
        self.current_state[6:9] = angular_velocity_base.copy()  # Base angular velocity
        self.current_state[9:12] = linear_velocity_base.copy()  # Base linear velocity
        self.current_state[12:24] = joint_angles.copy()  # Joint positions
        self.current_state[24:36] = joint_velocities.copy()  # Joint velocities

        # Update last values
        self.last_orientation = R_base_world
        self.last_position = base_position.copy()
        self.last_time = current_time

        if self.last_velocity_base is None:
            self.last_velocity_base = linear_velocity_base.copy()

        contact_bodies = ["FL_calf", "FR_calf", "RL_calf", "RR_calf"]

        contacts = [False for _ in range(4)]

        with lock:
            for i in range(data.ncon):  # ncon is the number of detected contacts
                contact = data.contact[i]
                geom1 = model.geom(contact.geom1)  # First geom involved in the contact
                geom2 = model.geom(contact.geom2)  # Second geom involved in the contact
                geom1_name = geom1.name
                geom2_name = geom2.name

                if geom1_name == "FL" or geom2_name == "FL":
                    contacts[0] = True

                if geom1_name == "RL" or geom2_name == "RL":
                    contacts[1] = True

                if geom1_name == "FR" or geom2_name == "FR":
                    contacts[2] = True

                if geom1_name == "RR" or geom2_name == "RR":
                    contacts[3] = True

        print(f"contacts: {contacts}")
        acceleration_base = (linear_velocity_base - self.last_velocity_base) / dt if dt > 0 else np.zeros(3)
        self.last_velocity_base = linear_velocity_base.copy()

        acceleration_base = acceleration_base + R_base_world @ np.array([0.0, 0.0, 9.81])

        print("Current velocity base: ", linear_velocity_base)
        self.ekf.update(
            current_time,
            dt,
            base_quat,
            joint_angles,
            joint_velocities,
            acceleration_base,
            angular_velocity_base,
            contacts,
        )
        print("Predicted position base: ", self.ekf.getBasePosition())
        print("Actual position base: ", base_position)
        # print("Predicted velocity base: ", self.ekf.getBaseVelocity())
        print()

        # print(f"state: {self.kalman_filter.getState()[3:6]}")

        self.current_state[3:6] = self.ekf.getBasePosition()
        self.current_state[9:12] = R_base_world @ self.ekf.getBaseVelocity()

        return self.current_state

    def getLatestRbdStamp(self):
        return time.time()

    def getContactFlags(self):
        # For now returning False for all 4 feet contacts
        # This could be enhanced by checking actual contact states in Mujoco
        return [False] * 4


class DummyCommandPublisher(CommandPublisher):
    def __init__(self, model, data):
        super().__init__()
        self.last_time = time.time()
        self.publish_count = 0
        self.start_time = time.time()
        self.model = model
        self.data = data

    def publish(self, commands):
        global kp
        global kd
        global desired_joint_angles
        kp = commands[0].kp
        kd = commands[0].kd
        for command in commands:
            desired_joint_angles[joint2idx[command.joint_name]] = command.desired_position


class DummyChangeControllerSubscriber(ChangeControllerSubscriber):
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


class DummyReferenceVelocityGenerator(ReferenceVelocityGenerator):
    def __init__(self, ui_controller: UIController):
        super().__init__()
        self.ui_controller = ui_controller

    def getReferenceVelocity(self, time, dt):
        ref = ReferenceVelocity()
        ref.velocity_x = self.ui_controller.linear_x
        ref.velocity_y = self.ui_controller.linear_y
        ref.yaw_rate = self.ui_controller.angular_z
        return ref


scene_path = "/home/kuba/Documents/mujoco_menagerie/unitree_go2/scene.xml"
# scene_path = "/home/kuba/Documents/mujoco_menagerie/anybotics_anymal_c/scene.xml"
model = mujoco.MjModel.from_xml_path(scene_path)
model.opt.timestep = 0.0025
data = mujoco.MjData(model)

print("Joint names:")
for i in range(1, 12 + 1):  # first joint is a virtual world->base_link joint
    print(f"  ({i})\t{mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)}")

sit_joint_angles = np.array([0.0, 0.806, -1.802, 0.0, 0.806, -1.802, 0.0, 0.996, -1.802, 0.0, 0.996, -1.802])
stand_joint_angles = np.array([0.0, 0.806, -1.802, 0.0, 0.806, -1.802, 0.0, 0.996, -1.802, 0.0, 0.996, -1.802])

# stand_joint_angles = np.array([0.0, 0.4, -0.8, 0.0, -0.4, 0.8, 0.0, 0.4, -0.8, 0.0, -0.4, 0.8])
# sit_joint_angles = np.array([0.0, 1.5, -2.6, 0.0, -1.5, 2.6, 0.0, 1.5, -2.6, 0.0, -1.5, 2.6])
# stand_joint_angles[3:6], stand_joint_angles[6:9] = sit_joint_angles[3:6], sit_joint_angles[6:9]
# sit_joint_angles[3:6], sit_joint_angles[6:9] = stand_joint_angles[3:6], stand_joint_angles[6:9]

data.qpos[-12:] = sit_joint_angles
desired_joint_angles = stand_joint_angles.copy()

window = mujoco.viewer.launch_passive(model, data)
lock = threading.Lock()


subscriber = DummyStateSubscriber(model, data)
publisher = DummyCommandPublisher(model, data)
controller_sub = DummyChangeControllerSubscriber()

rerun_logger = RerunLoggerNode(subscriber)


ui_controller = UIController(
    stand_callback=controller_sub.stand_callback,
    sit_callback=controller_sub.sit_callback,
    bob_callback=controller_sub.bob_callback,
)
ref_vel_gen = DummyReferenceVelocityGenerator(ui_controller)

tbai_python.write_init_time()

central_controller = tbai_python.CentralController.create(subscriber, publisher, controller_sub)


def callback(currentTime, dt):
    print(f"currentTime: {currentTime}, dt: {dt}")


central_controller.add_bob_controller(subscriber, ref_vel_gen, rerun_logger.visualize_callback)
central_controller.add_static_controller(subscriber, rerun_logger.visualize_callback)

viewer_thread = threading.Thread(target=viewer_fn, args=(window, lock, 1 / 30))
viewer_thread.start()

physics_thread = threading.Thread(target=physics_fn, args=(model, data, lock, model.opt.timestep, subscriber))
physics_thread.start()

central_controller.startThread()

try:
    ui_controller.run()
except KeyboardInterrupt:
    running = False
    print("Stopping threads")
    rerun_store("go2_robot.rrd")
    central_controller.stopThread()
    viewer_thread.join()
    physics_thread.join()
