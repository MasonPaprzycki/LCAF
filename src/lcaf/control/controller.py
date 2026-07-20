import time

from lcaf.telemetry.telemetry import Telemetry, SensorManager, TelemetrySnapshot
from lcaf.adaption.adaptive_planner import AdaptivePlanner
from motion_coordinator import MotionCoordinator 
from forge_brain import ForgeBrain

class Controller:

    def __init__(self):

        self.telemetry = Telemetry()
        self.motion = MotionCoordinator()
        self.planner = AdaptivePlanner()

        self.brain = ForgeBrain(motion=self.motion, planner=self.planner)

        self.telemetry.subscribe(self.brain.update)

        self.telemetry.subscribe(self.planner.update)

        self.rate = 50
        self.period = 1/self.rate

    def run(self):

        while True:

            start = time.perf_counter()

            # Machine acquisition
            self.motion.poll()

            # Create one truth packet
            snapshot = self.build_snapshot()

            # Broadcast heartbeat
            self.telemetry.publish(snapshot)

            # Brain decision
            self.brain.process_state_machine()

            elapsed = time.perf_counter() - start

            sleep = self.period - elapsed

            if sleep > 0:
                time.sleep(sleep)

    def build_snapshot(self):

        interface = self.motion.interface

        interface.update()

        return TelemetrySnapshot(

            timestamp=time.time(),

            machine_enabled = interface.machine_enabled(),

            machine_homed = interface.all_homed(),

            estop = interface.estop(),

            runtime = time.time() - self.brain.state.start_time,

            forge_mode = self.brain.state.forge_mode.name,

            motion_state = self.brain.state.motion_state.name,

            billet_temperature = None
        )