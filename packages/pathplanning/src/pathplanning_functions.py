#!/usr/bin/env python3

import rospy
from enum import Enum, auto
from typing import Tuple

# Gains for heading PID control
HEADING_KP = 0.3
HEADING_KI = 0.05
HEADING_KD = 0.028

class State(Enum):
    SCANNING = auto()        # scanning for objects
    # DETECTED_ANY = auto()  # an object is found, looking for the desired one, rotate around until desired object is identified
    IDENTIFIED = auto()      # desired object identified, drive towards it
    CAPTURED = auto()        # desired object captured within the "grabber"
    DELIVERING = auto()      # desired object is being delivered
    DELIVERED = auto()       # desired object is sucessfully delivered

class StateMachine:
    def __init__(self):
        self.state = State.SCANNING

    def transition(self, new_state: State):
        self.state = new_state


def scanning(car_control_msg, new_state, duckiedata, current_obj, v):
    # Turning until desired object is detected
    # rospy.loginfo("SCANNING")
    car_control_msg.v = 0
    car_control_msg.omega = v

    for i in range(round(duckiedata[0])):
    # compare ids of detected objects with desired object id:
        if duckiedata[i*3+3] == current_obj:
            new_state = State.IDENTIFIED
            car_control_msg.v = 0
            car_control_msg.omega = 0
            rospy.loginfo("desired object found")

    return car_control_msg, new_state


# def detected_any(car_control_msg, new_state, duckiedata, current_obj, direction):

#     rospy.loginfo("DETECTED_ANY")
    
#     for i in range(round(duckiedata[0])):
#         # compare ids of detected objects with desired object id:
#         if duckiedata[i*3+3] == current_obj:
#             new_state = State.IDENTIFIED
#             rospy.loginfo("setting new state identified")

#     if new_state == State.IDENTIFIED:
#         # if found, stop
#         car_control_msg.v = 0
#         car_control_msg.omega = 0
#         direction = 0
#     else:
#         # if not found, look around
#         if direction == 0:
#             car_control_msg.v = 0
#             car_control_msg.omega = 0.05
#             direction = 1
#         elif direction == 1:
#             car_control_msg.v = 0
#             car_control_msg.omega = -0.1
#             direction = 2
#         else:
#             car_control_msg.v = 0
#             car_control_msg.omega = 0.1
#             direction = 1

    
#     return car_control_msg, new_state, direction


def approach(car_control_msg, new_state, duckiedata, current_obj, prev_e, prev_int, delta_t, no_det_count):

    i = 0
    e = prev_e
    e_int = prev_int
    obj_ids = []
    for i in range(round(duckiedata[0])):
        # compare ids of detected objects with desired object id:
        if duckiedata[i*3+3] == current_obj:

            obj_ids.append(current_obj)
            no_det_count = 0       # reset 'not detected' counter because we have detected the desired object
            r = duckiedata[i*3+1]
            theta = duckiedata[i*3+2]

            theta_r = 0     # desired heading: angle zero, i.e. object straight ahead
            gains = (HEADING_KP, HEADING_KI, HEADING_KD)
            # PID controller for heading
            v = 0
            v, omega, e, e_int, e_der = PIDController(v, theta_r, theta, prev_e, prev_int, delta_t, gains)
            rospy.loginfo("PID values: omega, e, e_int, e_der: %.4f, %.4f, %.4f, %.4f", omega, e, e_int, e_der)
            
            if r > 0.28:
                rospy.loginfo("r>0.3")
                car_control_msg.v = 0.02
                car_control_msg.omega = omega
            elif r <= 0.28 and r > 0.15:
                rospy.loginfo("r>0.15")
                car_control_msg.v = 0.01
                car_control_msg.omega = omega
            else:
                rospy.loginfo("r<=0.15")
                car_control_msg.v = 0
                car_control_msg.omega = omega
                new_state = State.CAPTURED
    
    # desired object not found, stop. If not found for a couple of iterations, go back to scanning
    if current_obj not in obj_ids:
        no_det_count += 1
        car_control_msg.v = 0
        car_control_msg.omega = 0
        rospy.loginfo("Identified: object not found")
        if no_det_count >= 4:
            new_state = State.SCANNING
            rospy.loginfo("Identified: object not found, switch to scanning")
    

    # once within a certain distance, move forward a bit more, then change to captured state
    return car_control_msg, new_state, e, e_int, no_det_count


# def captured(car_control_msg, new_state, duckiedata, approach_count, dest_obj):

#     rospy.loginfo("CAPTURED")
#     # just stop for now. should rotate around 180° until destination object is detected
#     car_control_msg.v = 0
#     car_control_msg.omega = 0

#     if approach_count <= 5:
#         # If not close enough yet, go straight
#         approach_count += 1
#         car_control_msg.v = 0.02
#         car_control_msg.omega = 0
#     else:
#         car_control_msg, new_state = scanning(car_control_msg, new_state, duckiedata, dest_obj)

#         if new_state == State.IDENTIFIED:
#             new_state = State.DELIVERING

#     return car_control_msg, new_state, approach_count


# def delivering(car_control_msg, new_state, duckiedata, dest_obj, prev_e, prev_int, delta_t, no_det_count):
    
    
#     car_control_msg, new_state, e, e_int, no_det_count = approach(car_control_msg, new_state, duckiedata,
#                                                                   dest_obj, prev_e, prev_int, delta_t, no_det_count)

#     return car_control_msg, new_state, e, e_int, no_det_count


def delivered(car_control_msg, new_state):

    car_control_msg.v = 0
    car_control_msg.omega = 0

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
    e_int = max(min(e_int, 0.1), -0.1)

    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    # Kp = 5      # 15
    # Ki = 0.2    # 1
    # Kd = 0.1    # 0.2
    (Kp, Ki, Kd) = gains

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der

    return v_0, omega, e, e_int, e_der