Modeling, design, and implementation of an autonomous quadrotor. The intent of this project is to create a quadrotor that can carry a payload of 2.5 kg and, given inertial coordinates, navigate to the coordinates without requiring user input.
The quadrotor will be designed from scratch without using Electronic Speed Controllers (ESCs) for the motors or prebuilt software for flight control. 

The system architecture generally consists of a cascade of 3 controllers. At the lowest level, a Dynamic Inversion Model Reference Adaptive Controller (DIMRAC) is used to perform reference tracking with the angular velocity of an individual motor. This approach is
chosen as, ignoring electrical transients, the response of a brushed DC motor can be approximated by a first order differential equation. This makes the indirect method of MRAC where the adaptive gains are used to estimate the state and input matrices well suited.
The hardware implementation of this is with an Arduino Giga microcontroller sending Pulse Width Modulation (PWM) signals to a 2N2222 NPN transistor that is connected to the base of a NTE2536 NPN transistor in series with a  

of quadrotor dynamics performed in Matlab/SIMULINK. Control is split between an inner loop for the quadrotor attitude and an outer loop for the inertial position.

