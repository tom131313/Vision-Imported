## This Project utilizes FRC-related vision systems to detect and decode AprilTags in order to compute the robot position and drive to selected game targets.

### It is intended as a quick start for FRC teams relatively new to AprilTag and robot pose calculations. Purchased equipment can yield best results but a free solution (plus a $50 USB camera) is provided (pure use of WPILib classes).

This project is a wrapper called VisionContainer for the included three lower level wrappers for ControllerVision (WPILib), PhotonVision, and LimelightVision.

Three example uses of Vision using different targeting strategies are included as commands:
1. Turn the robot to a target and drive to a specified distance from the target (yaw and pitch of the robot
 wrt the AprilTag)
2. Drive the robot to a specified field position (3-D pose of the robot in the field)
  This (swerve) drive command is presented in two forms:

    a. use of three PID controllers to achieve the target position goal with a stub drivetrain receiving the motor commands

    b. proposed, but untested as packaged herein, PathPlanner on the fly command to drive to position

3. Drive the robot to a specified position relative to the target (Transform3d). This is similar to example #2 but instead of
using field coordinates it uses a displacement from a target.

These example commands are limited to aligning with a single selected target near FRC 2025 ReefScape reef AprilTag #10. Teams
provide their own game specific targeting code.

Completed team code should include the robot drivetrain, odometry and a gyro is extremely useful for pose accuracy and
drivability of the robot.

PID and timeout parameters must be tuned for each robot/drivetrain. Examples are for the hand-driven camera.

Read the comments at the top of Robot.java and the comments for setting up the three vision systems in their respective classes.

This project runs for all three vision systems in simulation mode or on a roboRIO v1.

The only requirements to run this minimal example in simulation mode (without a roboRIO) are a DriverStation PC and camera. (A
LimelightVision device must be purchased to use that device and then some method must be devised to connect the PC and the LL to
the same network.) The WPILib AprilTag and pose calculations are included or the free of charge PhotonVision may be downloaded
and simply run on the PC. A real roboRIO v1 may be used for running on a real robot.

An XBox controller is assumed but that can be easily changed; any digital inputs or other schemes can be used to trigger the
alignment commands.

An AprilTag #10 is assumed. Either change the number in the code or print or display on a screen the tag to be detected.

For 3-D pose calculations the camera must have its calibration data for accuracy. Guesstimates are fine for rough experimentation.

A purchased LimelightVision device may be used. A purchased co-processor for PhotonVision may be used.

For competitive use of the WPILib vision, a roboRIO v1 may be used only for turn to an angle targeting (3-D works but little time
remains for other processes). A roboRIO v2 is required for competitive use of the WPILib 3-D pose.

A recommended high quality, low cost vision system is free-of-charge PhotonVision on an inexpensive, recommended coprocessor. The
high quality LimelightVision is preferred by many teams for its simplicity but it costs more.