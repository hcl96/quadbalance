# quadbalance

PyQuadBalance

The ultimate flying controls for your quadcopter.
Designed with Open Source hardware and software, install this into your Raspberry Flight Computer to get started.

Connect to your RaspberryPi4 via WIFI and clone this repo to get started

PyQuadBalance will balance your quadcopter and allow you to control it via keyboard commannds:

               W(Move Forward)                 P(Power Up)
       A(Move Left)  D(Move Right)           L(Power Down)
               S(Move Back)             

PyQuadBalance runs a LQG controller with MPU9250 for roll and pitch control
The Kalman filter filters noise and vibrations at 100Hz to optimally estimate roll and pitch angles
The LQR tuned at equivalent of PD gain of 120,40 feedback the motors to achieve balance


