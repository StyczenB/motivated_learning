#!/usr/bin/env python3
import rospy
from typing import List
from robot_msgs.msg import MapMsg, FieldMsg, ChargerStateMsg, ChargersMsg
from environment.global_map_manager import Field
from geometry_msgs.msg import Point


class Charger:
    CHARGER_MAX_VAL = 1.0
    CHARGER_MIN_VAL = 0.0
    CHARGING_INCREMENT = 0.001

    def __init__(self, x: int, y: int, name: str):
        self._x: int = x
        self._y: int = y
        self._charger_value: float = Charger.CHARGER_MAX_VAL
        self._name: str = name

    @property
    def x(self) -> int:
        return self._x
    
    @property
    def y(self) -> int:
        return self._y

    @property
    def name(self) -> str:
        return self._name

    @property
    def charger_value(self) -> float:
        return self._charger_value

    def draw_energy(self) -> float:
        if self._charger_value < 0.1:
            return 0
        energy = self._charger_value * 0.005
        self._charger_value -= energy
        return energy

    def charging(self):
        self._charger_value += Charger.CHARGING_INCREMENT
        self._charger_value = max(min(self._charger_value, Charger.CHARGER_MAX_VAL), Charger.CHARGER_MIN_VAL)

    def __str__(self) -> str:
        return f'name: {self.name}, x: {self.x}, y: {self.y} -> charger value: {self.charger_value}\n'

    def get_charger_state_msg(self) -> ChargerStateMsg:
        return ChargerStateMsg(coords=Point(x=self.x, y=self.y), charger_value=self.charger_value, name=self.name)


class ChargersManager:
    def __init__(self):
        self._chargers: List[Charger] = []
        self._chargers_pub = rospy.Publisher('chargers', ChargersMsg, queue_size=1, latch=True)
        self.init_chargers()

    def init_chargers(self):
        rospy.loginfo('Waiting for \'global_world_map\' topic...')
        global_map = rospy.wait_for_message('global_world_map', MapMsg)
        rospy.loginfo('...topic \'global_world_map\' available')
        for field in global_map.fields:
            if field.type == Field.CHARGER:
                charger = Charger(x=field.coords.x, y=field.coords.y, name=field.name)
                self._chargers.append(charger)
        self.publish()

    def step(self):
        for charger in self._chargers:
            charger.charging()
        self.publish()

    def get_charger(self, x: int, y: int) -> Charger:
        for charger in self._chargers:
            if charger.x == x and charger.y == y:
                return charger
        return None

    def __str__(self):
        out = 'Chargers:\n'
        for charger in self._chargers:
            out += str(charger)
        return out

    def publish(self):
        chargers_msg = ChargersMsg()
        chargers_msg.header.stamp = rospy.Time().now()
        for charger in self._chargers:
            chargers_msg.chargers.append(charger.get_charger_state_msg())
        self._chargers_pub.publish(chargers_msg)
