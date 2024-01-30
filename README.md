Modeling, design, and implementation of an autonomous quadrotor. The intent of this project is to create a quadrotor that can carry a payload of 2.5 kg and, given inertial coordinates, navigate to the coordinates without requiring user input.
The quadrotor will be designed from scratch without using Electronic Speed Controllers (ESCs) for the motors or prebuilt software for flight control. 

The system architecture consists of a cascade of 3 controllers: 

  At the lowest level, a Dynamic Inversion Model Reference Adaptive Controller (DIMRAC) is used to perform reference tracking with the angular rate of an individual motor. This approach is
  chosen as, ignoring electrical transients, the response of a brushed DC motor can be approximated by a first order differential equation. This makes the indirect method of MRAC where the adaptive gains are used to estimate the state and input matrices well suited.
  The hardware implementation of this is with an Arduino Giga microcontroller sending Pulse Width Modulation (PWM) signals to a 2N2222 NPN transistor that is connected to the base of a NTE2536 NPN transistor in series with a RS555-EN 24V DC motor. The motor is
  outfitted with a 2 channel hall effect encoder that is used to measure motor angular rate. 
  
  At the intermediate level optimal control is implemented with a Linear Quadratic Gaussian (LQG) controller. This controller used a servo mechanism Linear Quadratic Regulator (LQR) for disturbance rejection and output tracking, and a Kalman filter for state
  estimation. Because there were no requirements for performing complex maneuvers it was acceptable to linearize the quadrotor dynamics around a hover operating point. This allows the controller and estimator gains to be solved offline and greatly simplifies 
  the design process. This controller accepts quadrotor euler angles as input and outputs desired torques which can be decomposed into motor angular rates. This decomposition is done using emperical data on propeller performance provided by the manufacturer 
  APC Propellers that is stored in a lookup table and retrieved depending on the most recent motor angular rate. For sensing quadrotor body angular rates and calculating euler angles a BMI088 6 axis accelerometer/gyro is connected to the Arduino Giga.

  At the highest level a LQG controller is again used for the same reasons as at the intermediate level. Here inertial coordinates are provided as reference inputs by the user and thrust and quadrotor euler angles are output to the intermediate level controller. 
  A PA1616S GPS module is used to measure inertial position states with the measurements of the BMI088 accelerometers being used as inputs to the system. 

A 22.2V 2900mAh 6S Lithium Polymer battery is used to power the 4 motors, it enables a flight time of 8 minutes. A 6V Nickel - Metal Hydride (NiMH) battery is used to power the Arduino Giga, PA1616S, BMI088, 4 encoders, and 4 2N2222 transistors. The propeller is
a 10x4.5MR-B4 from APC propellers. It is capable of generating lb thrust at rpm , APC propellers provides extensive emperical test data for each of their propellers. The frame is made of pine wood.
