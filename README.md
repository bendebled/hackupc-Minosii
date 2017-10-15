# Minosii

Nowadays, most remote control are still based with old buttons and joysticks. It is time to use more intuitive, fun, and safe remote. The type of remote controls are to be taken seriously because they are very important in certain applications such as dismantling bombs, managing hazardous elements, flying drones,...

Minosii aims to provide a way to substitute traditional remotes by the use of multiple accelerometers and gyroscopes. 

To realise this project, we used:

- 2x 9-axis sensor (Gyro + Accelerometer + Compass) : MPU9250
- 1x microcontroller : Arduino Micro
- 1x transceiver : nRF24L01

The module on the wrist consist of a microcontroller, a transceiver and the 9-axis sensor used as an axis reference. The 9-axis sensor remaining is on the fingertip. 

The principle of the protoype is quite simple, the microcontroller reads the value of the two 9-axis sensor and process those value to generate a 3D model of the movement. A simplification of that 3D model is then sent via the transceiver.