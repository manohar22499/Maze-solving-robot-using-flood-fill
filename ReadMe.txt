
#Robot Maze Solver

The Arduino code provides a program which helps the robot to solve maze. The maze size is given in ROWS and COLS.Each unit cells has a unit size defined my the user in the code.These algorthim follows the rules from the micromouse compitition.

Hardware 

1. Microcontroller - 1(Ardunino UNO)
2. IR sensors - 2 (WWZMDiB IR Infrared Sensor 3-Wire Reflective Photoelectric Module )
3. Ultrasonic sensor -1 (HC-SR04 Ultrasonic Module Distance Measuring Transducer Sensor)
4. Motor driver -1 (DRV 8833 motor driver)
5. Encoders-2 (Teyleten Robot IR Infrared Slotted Optical Optocoupler Module Speed Measuring Sensor)
5. Chassis with motor and wheels -1 
6. Jumper wires (Male and Female)


How to run code

1. Install Visual studio 
2. Decide on the size of maze ,direction of the robot and intial postion in the maze
3. Decide on the destination and size of your wheeldiameter
4. Update the value in the code as commented
5. Add Platform.io extension in Install Visual studio 
6. Install TimerOne,limits libraries
7. Connect with the robot with USB
8. Upload the code


Installation

1. Attach IR sensors at the right and left of the robot chassis (left-Pin 12)(Right-Pin 11)
2. Attach Ultra sonic sensor at the front(trig Pin 7)(echo Pin 4)
3. Connect Motor driver (supply of 6V) with the motors (EEP Pin 13)(In1 Pin 9 :In2 Pin 10 Left motor)(In3 Pin 5 :In4 Pin 6 Right motor)
4. Connect Encoders over the disk right and left wheel (out Pin 2- Left) (out Pin 3- right)( for ISR )
5. Ardiuno to 9V supply
6. Motor driver to 6V supply



Features

1. Floodfill algorithm is used where a Queue is used to generate matrix setting the destination as 0
2. Uses IR sensors for side wall detection
3. Ultra sonic sensor for front wall detection
4. Maze constructor is used to store horizontal walls, vertial walls(0 if no wall 1 if wall) and maze distances 
5. ISR are used for Encoders
6. Robot checks for minimum value in the adjacent cells and moves to the cells with lowest maze.distance  
7. Once the Robot encounter the wall it update the maze.distance considering the wall in a specific direction


Logic

1. Set up the maze, sensors, motors, and variables.
2. Using IR and ultrasonic sensors to detect and update walls in the maze in horizontal walls and vertical walls.
3. Movement Function which controls robot's linear and rotational movements.
4. Navigation & Decision Making function chooses paths  based on updated maze information(moving to towards lowest value).
5. In loop,it continuously updates the maze, robot's position, walls, and path towards the goal.


How to use it

Place the robot at the initial position then connect it to the power supply 1st motor driver supply then the microcontroller.Let the robot map the maze.




Limitation

Hardware is the limitation.As code is designed for educational  purposes.left and right straight and back must be calibrated. Ensure your robot's hardware is compatible with the code specifications. Adjust sensor  and motor settings as needed for your specific setup.