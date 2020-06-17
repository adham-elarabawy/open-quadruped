# OpenQuadruped
An open-source 3D-printed quadrupedal robot. Motion Algorithms for dynamic walking gaits, Artificial Intelligence for dynamic terrain mapping &amp; obstacle avoidance, Self-Balancing.

## Hardware
I made a custom pcb board to control the position and speed of 12 servos simultaneously, as well as itnerface with all of the sensors.

You can find the gerber files for the custom pcb in the [hardware folder](https://github.com/adham-elarabawy/OpenQuadruped/tree/master/src/hardware) in this repository.

There are two versions: one that uses one power source to power all 12 servos and one that uses two power sources to power all 12 servos (in case of 2 batteries or two lower-current UBECs/Voltage regulators). Both version offer access to the teensy's serial pins, two I2C breakouts (in case you want to connect sensors) and a regulated 5V rail. The board uses 2mm wide traces + 2 ground planes in order to properly dissipate heat for high currents.

![SinglePCB](https://github.com/adham-elarabawy/OpenQuadruped/blob/master/hardware/SinglePCB.png)
![DoublePCB](https://github.com/adham-elarabawy/OpenQuadruped/blob/master/hardware/DoublePCB.png)

## Visualization Usage
To try the visualization tool out, you'll need to run the [animate.py](https://github.com/adham-elarabawy/OpenQuadruped/blob/master/src/visualization/animate.py) python file with the proper libraries installed. 

You can then use keyboard controls: use x, y, z, a, p, r to select (x axis, y axis, z axis, yaw, pitch, roll), and then the up and down buttons to increment the selected position. If you click '1' on your keyboard, it will reset the position. 

Right now, if you try to go to an impossible pose that would result in collisions, the body will do some weird things. If that happens, just click "1" on your keyboard to reset the position. 

*Note: Pitch and Roll are currently not working. I am currently trying to fix that.*

## Demos
[![IK Model Demo](https://img.youtube.com/vi/cCMvCH0m9TA/0.jpg)](https://www.youtube.com/watch?v=cCMvCH0m9TA)
[![IK Model Visualization](https://img.youtube.com/vi/LBjqJVEXwhM/0.jpg)](https://www.youtube.com/watch?v=LBjqJVEXwhM)

