package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * This Project is a wrapper called {@link VisionContainer} for three lower level wrappers {@link ControllerVision}
 * {@link PhotonVision} {@link LimelightVision} of three complex vision systems - WPILib, PhotonVision, and LimelightVision.
 * 
 * After selecting and configuring the desired vision system as described far below, use the robot pose as in an example command
 * {@link AlignToReefFieldRelativePose3D} or {@link AlignToReefTagRelativeArcade2D}
 * 
 * <p>See {@link VisionContainer} for a brief example of code to use the RobotPose provided by that class and how to
 * instantiate each of the vision systems.
 * 
 * <p>Each of the three wrappers for the three vision system also may be used without benefit of the simple VisionContainer wrapper.
 * Doing so allows use of more methods in the individual vision system wrapper. And there is no requirement to use any of these wrappers.
 * Each vision system has a complex set of methods to perform a lot of functions. These wrappers make it easier to get started quickly.
 * 
 * <p>PhotonVision and LimelightVision each have examples of their usage on their web sites. Most teams have Vision
 * code; there are hundreds of varieties. This code helps beginner or novice teams get started quicker and gives hints of some
 * possibilities to use vision and commands.
 * 
 * <p>Presentation of three Vision systems:
 *   <p>WPILib Vision example (known herein as ControllerVision) with an Example wrapper consistent with the other two wrappers
 *   <p>Example wrapper for PhotonVision
 *   <p>Example wrapper for LimelightVision
 * <p>Each system puts robot pose information to the NetworkTables and that can be viewed in AdvantageScope or other viewers.
 * 
 * <p>These three Vision routines each can detect AprilTags and compute a robot pose. Select the one to run in
 * {@link RobotContainer#RobotContainer()}
 * <p>This project may be used in whole by selecting which one of the three vision systems to be demonstrated.
 * 
 * <p>Each of the Visions Systems must be configured for the "camera" name and minimum parameters needed for each system.
 * <p>Each has its mounted camera location wrt the robot that must be provided by the team (example default provided). {@link VisionContainer}
 * <p>The ControllerVision (completely run on the roboRIO) must also have the chosen camera calibration parameters specified herein
 * (a few examples provided). {@link ControllerVision#ControllerVision()}
 * <p>ControllerVision arbitrarily selects to use the robot pose of the lowest AprilTag number if there are multiple
 * tags in view in a frame. That is likely wrong a lot of the time.
 * Some smarts need to be added so targeting goes to the right target {@link ControllerVision#getPose3d()} and below it.
 * <p>Another targeting strategy is to use a 3-D position of the robot wrt a target without regard to the coordinates on the field.
 * That target-relative position is somewhat similar but different than the field-relative pose provided in this project. It wouldn't
 * be hard to add that function is desired. Many examples on the Internet tend to show that method of targeting and it is fully
 * supported by PhotonVision and LimelightVision. It can be added to this project's ControllerVision {@link AcquireRobotPose#run()}
 * by revealing the existing object "tagFacingCameraFrame" and that may be adjusted for the object "cameraInRobotFrame".
 * <p>Use of PhotonVision or LimelightVision require that those devices and software be setup as instructed in their
 *  respective documentation. They are mostly configured through their respective "dashboards."
 * <p>Use of Limelight MegaTag2 requires the robot heading (gyro value) be sent to LL on each iteration. {@link LimelightVision#update()}
 * <p>Selection of the best pose from LimelightVision requires filtering. Simple example is presented. {@link LimelightVision#update()}
 * <p>Selection of the very best pose estimate from PhotonVision requires using {@link PhotonVision#getEstimatedGlobalPose(Pose2d)}.
 * A dummy stub is included to give a starting hint of what could be done but wasn't in this project. Otherwise, the simpler
 * "bestTarget" as used in this project is a good approximation.
 * <p>Use of the WPILib PoseEstimator requires values from this vision targeting project be merged with the gyro
 * and wheel odometry positions.
 * 
 * <p>This demonstration project may be run in simulation mode or on a roboRIO. The cameras are not simulated. That is, no roboRIO is
 * required but the roboRIO simulation may be used with one of the following attached (USB or network) to the PC running simulation:
 * <p>a camera attached to the simulation PC,
 * <p>a LimelightVision box attached to the same network as the simulation PC,
 * <p>or PhotonVision running on some device - the simulation PC or a coprocessor attached to the same network as the simulation PC
 *
 * <p>For team's customized use, it is fairly easy to copy the code of one or more of the vision systems into a team's own project.
 * <p>{@link VisionContainer} extends {@link SubsystemBase} solely for the purpose of the automatically executed
 * {@link SubsystemBase#periodic()} method. The extends Subsystem may be removed and the periodic() method run in say
 * {@link Robot#robotPeriodic()}
 * 
 * <p>Two example uses of Vision are included as commands:
 * <p> 1. Turn the robot to a target and drive to a specified distance from the target (yaw and pitch of the robot
 * wrt the AprilTag) {@link AlignToReefTagRelativeArcade2D} (activated by XBox controller B button {@link RobotContainer#configBButton()})
 * <p>   Heed the caution in this example targeting command about the limitations of the distance calculation and a circumvention.
 * <p> 2. Drive the robot to a specified field position (3-D pose of the robot in the field) {@link AlignToReefFieldRelativePose3D}
 * (activated by XBox controller A button {@link RobotContainer#configAButton()})
 * <p> This (swerve) drive command is presented in two forms:
 * <p>   a. use of three PID controllers to achieve the target position goal with a stub drivetrain receiving the motor commands
 * <p>   b. proposed, but untested as packaged herein, PathPlanner on the fly command to drive to position
 *
 * <p>  These example commands are limited to aligning with a single selected target near 2025 Reefscape reef AprilTag #10.
 * <p>  Completed team code should include the robot drivetrain odometry and a gyro is extremely useful
 * for pose accuracy and drivability of the robot.
 * <p>  PID and timeout parameters must be tuned for each robot/drivetrain. Examples are for the hand-driven camera.
 * 
 * <p>This code has been tested with Limelight 2+, PhotonVision 2026 beta 1 on RPi 4 B, and WPILib 2025.3.2
 * with both a roboRIO v1 and DriverStation PC simulation.
 * <p>Targeting commands were driven by a hand-held camera without benefit of a robot.
 * <p>Cameras used were Microsoft LifeCam HD-3000 (USB interface) and ArduCam UC-844 Rev. B (OV9281 camera sensor; USB interface).
 * <p>Gyro was simulated with a "0" heading.
 * <p>Drivetrain was crudely simulated by printing the commanded velocity and voltage values.
 * 
 * 
 * <p>-----Setting up simulation of the roboRIO on the PC-----
 * <p>Get the ip address of the PC running the simulation by command prompt program "ipconfig" (linux command is "ifconfig")
 * <p>Optional Start Outline Viewer on the PC and change Options/settings/team/IP 127.0.0.1 (or the PC IP address) - client nt4 mode and apply
 * <p>For PhotonVision start the device/program and change Settings Team Number/NetworkTables Server Address enter the PC IP address (the real one; not 127.0.0.1)
 * <p>For LimelightVision start the device and change Settings Custom NT Server IP: enter the PC IP address (the real one; not 127.0.0.1)
 * <p>For ControllerVision plugin the camera into the PC
 * <p>AdvantageScope start the program and select File / Connect to Simulator
 * <p>Start the "Simulate Robot Code" on the PC
 * <p>
 * <p>This is not simulating the camera device; that is not supported in this project. A real camera device
 * must be used. Only the roboRIO is being simulated.
 * 
 * <p>Example of camera simulation is available (with limitations) in PhotonVision only.
 * <p>--------------------------------------------------------
 * 
 * <p>Cameras that allow manual focus must be checked and focused. It's easy to focus a camera by eye or use the LimelightVision focus
 * pipeline for its own camera.
 * A Siemens Star can be used or any object at about the typical distance that the camera is to be used. Mark the lens holder where the
 * camera is focused and glue into place if it's prone to moving.
 * 
 * <p>Autofocus sounds like a good idea but some inexpensive cameras jitter violently trying to focus continuously especially while the
 * camera is moving (with the robot, of course). Autofocus likely is more harmful than helpful.
 * 
 * <p>Cameras need to be calibrated - creating a camera matrix at the desired resolution. Calibrate using PhotonVision and copy its
 * camera matrix from the downloaded setting file or try this online calibration database and calibrator https://www.calibdb.net/ .
 * LimelightVision includes a calibration process for its own camera.
 * 
 * If desired, unneeded vision code may be stripped out based on each file's use:
 *   AcquireAprilTag.java                    CV
 *   AcquireRobotPose.java                   CV
 *   AlignToReefFieldRelativePose3D.java     CV, PV, LV
 *   AlignToReefTagRelativeArcade2D.java     CV, PV, LV
 *   AprilTagsLocations.java                 CV, PV, LV (LV only by association with drive commands that may use the tag locations)
 *   CameraBase.java                         CV, PV, LV
 *   CommandSchedulerLog.java                CV, PV, LV
 *   ControllerVision.java                   CV
 *   Image.java                              CV
 *   LimelightHelpers.java                           LV
 *   LimelightVision.java                            LV
 *   Main.java                               CV, PV, LV
 *   PhotonVision.java                           PV                       
 *   Robot.java                              CV, PV, LV
 *   RobotContainer.java                     CV, PV, LV
 *   RobotPose.java                          CV, PV, LV
 *   SpikeFilter.java                        CV
 *   VisionContainer.java                    CV, PV, LV (four cases of the two undesired vision processes may be removed)
*/

@Logged
public class Robot extends TimedRobot {
    static
    {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }
    
    public Robot()
    {    
        new RobotContainer();
    }

    @Override
    public void robotPeriodic() // called last in the periodic loop despite always seen written first in Robot.java
    {
        CommandScheduler.getInstance().run();
    }
}
