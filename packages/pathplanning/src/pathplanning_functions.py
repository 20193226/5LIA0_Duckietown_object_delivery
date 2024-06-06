#!/usr/bin/env python3

import rospy
from enum import Enum, auto
from typing import Tuple

# Gains for heading PID control
HEADING_KP = 3
HEADING_KI = 0.01
HEADING_KD = 0.1

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

    def transition(self, new_state: State):
        self.state = new_state


def scanning(car_control_msg, new_state, duckiedata):
    # For now: going straight with constant speed until an object is detected
    #rospy.loginfo("SCANNING")
    car_control_msg.v = 0.05
    car_control_msg.omega = 0

    if duckiedata[0] > 0:
        new_state = State.DETECTED_ANY
        rospy.loginfo("setting new state detected any")

    return car_control_msg, new_state


def detected_any(car_control_msg, new_state, duckiedata, obj_sequence, current_obj_cnt, idx_curr_obj):

    rospy.loginfo("DETECTED_ANY")
    
    for i in range(round(duckiedata[0])):
        # compare ids of detected objects with desired object id:
        if duckiedata[i*3+3] == obj_sequence[current_obj_cnt]:
            idx_curr_obj = i
            new_state = State.IDENTIFIED
            rospy.loginfo("setting new state identified")

    if new_state != State.IDENTIFIED:
        # if not found, look around to find desired object (just stop for now)
        pass
        # car_control_msg.v = 0
        # car_control_msg.omega = 0
    else:
        pass
        # car_control_msg.v = 0
        # car_control_msg.omega = 0
    
    return car_control_msg, new_state, idx_curr_obj


def identified(car_control_msg, new_state, duckiedata, idx_curr_obj, prev_e, prev_int, delta_t):

    rospy.loginfo("IDENTIFIED")
    r = duckiedata[idx_curr_obj*3+1]
    theta = duckiedata[idx_curr_obj*3+2]
    # id = duckiedata[idx_curr_obj*3+3]

    theta_r = 0
    gains = (HEADING_KP, HEADING_KI, HEADING_KD)
    # PID controller for heading
    v = 0
    v, omega, e, e_int = PIDController(v, theta_r, theta, prev_e, prev_int, delta_t, gains)
    rospy.loginfo("PID values: v, omega, e, e_int: %.4f, %.4f, %.4f", omega, e, e_int)
    
    if r > 0.3:
        car_control_msg.v = 0.05
        car_control_msg.omega = omega
    elif r > 0.15:
        car_control_msg.v = 0.03
        car_control_msg.omega = omega
    else:
        car_control_msg.v = 0
        car_control_msg.omega = omega

    # once within a certain distance, move forward a bit more, then change to captured state
    return car_control_msg, new_state, e, e_int


def captured(car_control_msg, new_state):

    rospy.loginfo("CAPTURED")
    # just stop for now
    car_control_msg.v = 0
    car_control_msg.omega = 0
    return car_control_msg, new_state


def delivering(car_control_msg, new_state):
    return car_control_msg, new_state


def delivered(car_control_msg, new_state):
    return car_control_msg, new_state


def PIDController(v_0: float,
                  theta_ref: float,
                  theta_hat: float,
                  prev_e: float,
                  prev_int: float,
                  delta_t: float,
                  gains: Tuple[float, float, float]
                  ) -> Tuple[float, float, float, float]:
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        theta_ref (:double:) reference heading pose
        theta_hat (:double:) the current estiamted theta.
        prev_e (:double:) tracking error at previous iteration.
        prev_int (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
        gains (:Tuple:) PID controller gains
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    
   # Tracking error
    e = theta_ref - theta_hat

    # integral of the error
    e_int = prev_int + e*delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int, 0.5), -0.5)

    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    # Kp = 5      # 15
    # Ki = 0.2    # 1
    # Kd = 0.1    # 0.2
    (Kp, Ki, Kd) = gains

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der

    return v_0, omega, e, e_int