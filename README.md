# quadbalance

PyQuadBalance

The ultimate flying interface for your quadcopter is here.
Designed with Open Source hardware and software, PyQuadBalance is tested on Q450 quadcopter with Raspberry Pi 4.

If your RPi4 has been configured to support remote access via WIFI, you can control it directly via keyboard commands.
While a radio transmitter delivers the true RC experience, let's not underestimate the power of its CLI counterpart.

Designed to mimic the transmitter, the keyboard control layout is intuitive:


                    W(Move Forward)                                    P(Power Up)
      A(Move Left)                  D(Move Right)             
                    S(Move Back)                                  L(Power Down)
                    

PyQuadBalance will keep your quadcopter hovering in balance as you command it.

PyQuadBalance runs a LQG controller with MPU9250 for roll and pitch control
The Kalman filter filters noise and vibrations to faithfully estimate roll and pitch angles
The LQR tuned at equivalent of PD gain of 80,40 motor feedback to maintain balance at 100 times a second.

Geolocation feature is available as well. Simply plug in an Arduino GPS breakout to Tx/Rx pins on your RPi4 to enable station keeping. This will allow you to set up a virtual cage around the drone and prevent runaway. The GPS coordinates and altitude are displayed on the user console as well. Furthermore, GPS module significantly increases the accuracy of the Kalman filter. This will enable the Kalman to track location, altitude, velocity and improve performance of your drone. 


