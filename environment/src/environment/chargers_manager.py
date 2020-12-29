#!/usr/bin/env python3
import rospy
from typing import List
from robot_msgs.msg import MapMsg, FieldMsg, ChargerStateMsg, ChargersMsg
from robot_msgs.srv import DrawChargerEnergy, DrawChargerEnergyRequest, DrawChargerEnergyResponse
from .global_map_manager import Field
from geometry_msgs.msg import Point


class Charger:
    """
    Single charger class resposible to store information about position, name and current charge value.
    """

    CHARGER_MAX = 1.0
    CHARGER_MIN = 0.0
    CHARGING_INCREMENT = 0.001

    def __init__(self, x: int, y: int, name: str):
        self._x: int = x
        self._y: int = y
        self._charger_value: float = Charger.CHARGER_MAX
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
        self._charger_value = max(min(self._charger_value, Charger.CHARGER_MAX), Charger.CHARGER_MIN)

    def __str__(self) -> str:
        return f'name: {self.name}, x: {self.x}, y: {self.y} -> charger value: {self.charger_value}\n'

    def get_charger_state_msg(self) -> ChargerStateMsg:
        return ChargerStateMsg(coords=Point(x=self.x, y=self.y), charger_value=self.charger_value, name=self.name)


class ChargersManager:
    """
    Manager resposible for chargers on the map.
    """

    def __init__(self):
        self._chargers: List[Charger] = []
        self._chargers_pub = rospy.Publisher('chargers', ChargersMsg, queue_size=10, latch=True)
        self._get_charger_energy_srv = rospy.Service('draw_charger_energy', DrawChargerEnergy, self._draw_charger_energy_handler)
        self.init_chargers()

    def init_chargers(self):
        rospy.loginfo('Waiting for \'global_world_map\' topic...')
        global_map: MapMsg = rospy.wait_for_message('global_world_map', MapMsg)
        rospy.loginfo('...topic \'global_world_map\' available')
        for field in global_map.fields:
            if field.type == Field.CHARGER:
                charger = Charger(x=field.coords.x, y=field.coords.y, name=field.name)
                self._chargers.append(charger)
        self.publish()

    def _draw_charger_energy_handler(self, req: DrawChargerEnergyRequest) -> DrawChargerEnergyResponse:
        res = DrawChargerEnergyResponse()
        for charger in self._chargers:
            if charger.x == req.coords.x and charger.y == req.coords.y:
                res.energy = charger.draw_energy()
                break
        self.publish()
        return res

    def step(self):
        for charger in self._chargers:
            charger.charging()
        self.publish()

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


class ChargersManagerClient:
    def __init__(self):
        self._draw_charger_energy_handler = rospy.ServiceProxy('draw_charger_energy', DrawChargerEnergy)
        self._draw_charger_energy_handler.wait_for_service()

    def draw_energy_from_charger(self, x: int, y: int) -> float:
        req = DrawChargerEnergyRequest(coords=Point(x=x, y=y))
        res: DrawChargerEnergyResponse = self._draw_charger_energy_handler(req)
        return res.energy

    def get_charger_energy_level(self, x: int, y: int) -> float:
        print('reading one message')
        chargers: ChargersMsg = rospy.wait_for_message('chargers', ChargersMsg)
        print('...done reading one message')
        for charger in chargers.chargers:
            if charger.coords.x == x and charger.coords.y == y:
                return charger.charger_value
        return -1
