# Advanced-Line-Following-Robot

A submission for WRG. This robot features a PID controller that constantly adjust its position. Can be used with 8-15 arrays (more or less)

## How does it work?
1. Upon startup, the robot calibrates its sensors depending on its environments
2. After finishing the calibration, the robot begins its sensor streaming, which constantly reads the values of the IR sensors
3. While streaming, the robot decide for 4 things: To avoid a potential obstacle (BETA), to terminate its process, to blindly go forward, and turn. The robot will prioritize avoidinng the object, terminating its processes, blindly going forward, and turning respectively

## Caveats

- NOTHING may work in this repository as long as it is in development (and will most likely always remain as a prototype).
- Very gimmicky to use and callibrate

## Positive notes

- Easily adaptable to any other microcontroller (ESP32 preferred)
- Open source (yippee)