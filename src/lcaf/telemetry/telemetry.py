from dataclasses import dataclass

class SensorManager:

    def __init__(self):
        self.sensors = []

    def register(self, sensor):
        self.sensors.append(sensor)

    def update(self):
        for sensor in self.sensors:
            sensor.poll()

class Telemetry:

    def __init__(self):
        self.subscribers = []
        self.sensor_manager = SensorManager()

    def subscribe(self, callback):
        self.subscribers.append(callback)

    def publish(self, snapshot):
        self.sensor_manager.update()
        for callback in self.subscribers:
            callback(snapshot)


@dataclass(frozen=True)
class TelemetrySnapshot:
    """
    One heartbeat's worth of broadcast state (see docs/architecture.md,
    docs/state_machine.md). Field names here are the contract every
    subscriber (ForgeBrain.update(), AdaptivePlanner.update(), and any
    future one) reads by name -- keep them matched to what
    Controller.build_snapshot() actually provides.

    forge_mode/motion_state are ForgeBrain's own current state, included
    here only for subscribers other than ForgeBrain itself (e.g. a future
    dashboard/logger) -- ForgeBrain.update() does not read these two back
    into its own SystemState, since that would just be feeding its own
    output back into its own input every heartbeat.
    """
    timestamp: float
    machine_enabled: bool
    machine_homed: bool
    estop: bool
    runtime: float
    forge_mode: str
    motion_state: str
    billet_temperature: float | None

    #add sensor data