import tbai_python
import numpy as np

import time

from tbai_python import StateSubscriber, CommandPublisher, ChangeControllerSubscriber, ReferenceVelocity, ReferenceVelocityGenerator

import ctypes

class DummyStateSubscriber(StateSubscriber):
    def __init__(self):
        super().__init__()
        self.initialized = False

    def waitTillInitialized(self):
        self.initialized = True

    def getLatestRbdState(self):
        return np.zeros((36,), dtype=np.float64)

    def getLatestRbdStamp(self):
        return 0.0

    def getContactFlags(self):
        return [False] * 4

class DummyCommandPublisher(CommandPublisher):
    def __init__(self):
        super().__init__()
        self.last_time = time.time()
        self.publish_count = 0
        self.start_time = time.time()

    def publish(self, commands):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Count number of publish calls
        self.publish_count += 1
        elapsed = current_time - self.start_time
        
        # Print frequency every 100 calls
        if self.publish_count % 100 == 0:
            freq = self.publish_count / elapsed
            print(f"Publish frequency: {freq:.1f} Hz")
            self.start_time = current_time
            self.publish_count = 0
            
        return commands

class DummyChangeControllerSubscriber(ChangeControllerSubscriber):
    def __init__(self):
        super().__init__()
        self.callback = None

    def set_callback_function(self, callback):
        self.callback = callback

    def triggerCallbacks(self):
        if self.callback:
            self.callback("dummy_controller")

class DummyReferenceVelocityGenerator(ReferenceVelocityGenerator):
    def __init__(self):
        super().__init__()

    def getReferenceVelocity(self, time, dt):
        ref = ReferenceVelocity()
        ref.velocity_x = 0.0
        ref.velocity_y = 0.0
        ref.yaw_rate = 0.0
        return ref

import gc
import weakref

subscriber = DummyStateSubscriber()
publisher = DummyCommandPublisher()
controller_sub = DummyChangeControllerSubscriber()
ref_vel_gen = DummyReferenceVelocityGenerator()

central_controller = tbai_python.CentralController.create(
    subscriber,
    publisher,
    controller_sub
)

central_controller.add_bob_controller(subscriber, ref_vel_gen)
central_controller.start()
