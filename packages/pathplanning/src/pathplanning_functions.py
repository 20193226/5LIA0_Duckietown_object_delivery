#!/usr/bin/env python3

from enum import Enum, auto

class State(Enum):
    SCANNING = auto()        # no objects found yet, driving forward until we do
    DETECTED_ANY = auto()    # an object is found, looking for the desired one, rotate around until desired object is identified
    IDENTIFIED = auto()      # desired object identified, drive towards it
    CAPTURED = auto()        # desired object captured within the "grabber"
    DELIVERING = auto()      # desired object is being delivered
    DELIVERED = auto()       # desired object is sucessfully delivered

class StateMachine:
    def __init__(self):
        self.state = State.SCANNING

    def transition(self, new_state):
        self.state = new_state

def scanning(car_control_msg):
    car_control_msg.v = 0.2
    car_control_msg.omega = 0
    return car_control_msg

def detected_any(car_control_msg):
    car_control_msg.v = 0
    car_control_msg.omega = 0
    return car_control_msg