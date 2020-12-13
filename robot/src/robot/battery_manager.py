#!/usr/bin/env python3

class BatteryManager:
    BATTERY_MAX = 1
    BATTERY_MIN = 0
    MOVING_DISCHARGE = 0.995

    def __init__(self):
        self._battery_level = BatteryManager.BATTERY_MAX
        self._idle_discharge_coeff = 0.9995
        self._moving_discharge_coeff = BatteryManager.MOVING_DISCHARGE

    @property
    def level(self) -> float:
        return self._battery_level

    @property
    def moving_discharge_coeff(self) -> float:
        return self._moving_discharge_coeff

    def no_draining(self):
        self._moving_discharge_coeff = 1.0

    def reset_discharge(self):
        self._moving_discharge_coeff = BatteryManager.MOVING_DISCHARGE

    def idle_discharge(self):
        self._battery_level *= self._idle_discharge_coeff

    def moving_discharge(self):
        self._battery_level *= self._moving_discharge_coeff

    def update_battery(self, update_value: float):
        if self._battery_level + update_value < 0:
            rospy.logwarn('Battery level cannot be negative. Dropping update...')
        else:
            self._battery_level += update_value
        self._battery_level = min(max(BatteryManager.BATTERY_MIN, self._battery_level), BatteryManager.BATTERY_MAX)
