# Mavlink

The main.cpp allows an arduino to exchange telemetry data with pixhawk using standard mavlink messages. It allows the user to control the quadrotor movement through desired attitude inputs and displays quadcopters realtime data on Arduino Serial Console

User may enter following commands in Serial monitor to carry out afformentioned tasks:

Arm Rotor :   SET_ARM

Disarm Rotor: SET_DIARM

Switch Mode : SET_FLIGHT_MODE_MODENAME ( i.e SET_FLIGHT_MODE_STABIZE)
              Other modes are AUTO , ALTHOLD , LOITER , CIRCLE
              
Contol Attitude and Throttle Values:
    ROLL :    SET_ROLL 
    PITCH:    SET_PITCH
    THROTTLE: SET_THROTTLE
    YAW:      SET_YAW
