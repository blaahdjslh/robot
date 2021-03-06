# Robot 
Navigation Code for FEH Robotics.
## Important Notes
1. Tuning the robot's motors should all be done in reference to the back motor's clockwise speed.
2. Alignment using the photosensor is unwise, as it may trigger red or blue incorrectly.
3. Alignment with walls preferably uses the front of the robot, as it is already relatively flat with the wall, and a plate can easily be mounted there.
## To do list
1. Calibrate and validate translational movement in the cardinal directions.
2. Re-calibrate and validate the rotational movement after tuning the cardinal directions.
2. Implement and validate line based navigation.
3. Implement and validate wall based navigation (ramming or bump switches).
4. Implement and validate the RPS navigation system.
5. Implement and validate the method for flipping the burger.
6. Implement and validate the method for flipping the ice cream levers up and down.
7. Final Path Planning.
## Completed list
1. Basic Translational and Rotational Movement.
2. Identification of colors.
3. Identification of line positions.
4. Implement and validate the method for depositing the tray onto the trash can.
5. Implement and validate the robot arm ticket system.
## Future Improvementrs
1. Comprehensive Translational Movement, including uniform motion in every direction.
2. Comprehensive Rotational Movement, including rotating while translating in any direction.
3. Comprehensive fall back modes for each stage of the course.
4. Curve based path planning (exotic and unneeded)
