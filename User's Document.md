# Document for Users in Orbit Simulation

## General View

### Basic coordinate in this simulation project

This section will specificly discuss the references used in this project. Users are supposed to read this part carefully, eliminating misunderstanding while using this program.

Firstly, position and velocit use cartysian coordinate system. In this case, this program use array to record these vectors. The center of the position system is the center of the Earth. It will not move at all, not because of moon or sun.

Secondly, the rotation part takes anticlockwise as the positive direction, so as angular acceleration and angular velocity.

Thirdly, the attitude of the spacecraft is based on the velocity and angle of attack(AOA) to identify a specific attitude.