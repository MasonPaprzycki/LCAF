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
    timestamp: float
    motion_enabled: bool
    machine_homed: bool
    estop: bool
    runtime: float
    forge_mode: str
    motion_state: str
    billet_temperature: float | None

    #add sensor data 