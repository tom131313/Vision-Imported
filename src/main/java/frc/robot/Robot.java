/**
 * Presentation of three Vision routines:
 *   WPILib Vision example (known herein as ControllerVision) with a wrapper consistent with the other two wrappers
 *   Example wrapper for PhotonVision
 *   Example wrapper for LimelightVision
 * 
 * These three Vision routines each can detect AprilTags and compute a robot pose.
 * 
 * Two example uses of Vision are included as commands:
 *  1. Turn the robot to a target and drive to a specified distance from the target
 *  2. Drive the robot to a specified field position
 *     This drive command is presented in two forms:
 *      1. use of three PID controllers to achieve the target position goal
 *      2. proposed, but untested as packaged herein, PathPlanner swerve command on the fly to drive to position
 *
 *   These example commands are limited to aligning with a single selected target near AprilTag #10.
 *   Completed team code should include the true robot drivetrain odometry and a gyro is extremely useful
 *   for pose accuracy and drivability of the robot.
 * 
 * PhotonVision and LimelightVision each have examples of their usage. Most teams have Vision code; there are
 * hundreds of varieties. This code may help beginner or novice teams get started quicker.
 * 
 * <p>This code has been tested with Limelight 2+, PhotonVision 2026 beta 1 on RPi 4 B, and WPILib 2025.3.2
 * with both a roboRIO v1 and DriverStation PC simulation. Targeting commands were driven by a hand-held camera
 * without benefit of a robot. Cameras used were Microsoft LifeCam HD-3000 (USB interface) and ArduCam UC-844 Rev. B (OV9281 camera sensor; USB interface)
 */

/* 
Vision hardware and software - partial list:

    roboRIO + free WPILib software (WPILib sample detection and Pose calculation from this project) + $55 camera am-5749 (different or cheaper USB cameras may work - reserve the right to return dysfunctional ones)
        roboRIO v1 turn to angle only
        roboRIO v2 3-D pose capability

    limelightvision.io $450 integrated hardware and software

    photonvision.org free software and has some hardware recommendations

    https://danpeled.gitbook.io/synapse/ free vision software and has some hardware recommendations

    Hardware that may run free vision software such as PhotonVision:

        https://luma.vision/ $250 hardware box including camera

        https://first-rubik.github.io/ $109 am-5698 RUBIK Pi 3 Vision Bundle + $55 camera am-5749 +
            power supply https://www.pololu.com/product/4984 $19 -> https://www.amazon.com/dp/B0C8TBM2QM 2 for $17 -> USB C PD Input on RUBIK Pi
            see https://first-rubik.github.io/docs/power
            https://www.chiefdelphi.com/t/introducing-the-rubik-pi-3-apriltag-processing-object-detection/507648/
            other possibilities:
            https://www.amazon.com/DROK-Converter-Adjustable-Regulator-Transformer/dp/B0BQBXMH67?th=1
            https://www.amazon.com/dp/B0D7942HWP?th=1

        Raspberry Pi 3, 4, or 5 + $55 camera am-5749 + voltage regulator (some other USB and CSI cameras work)

        Orange Pi + camera + voltage regulator

    WPILib + roboRIO are "free" so why not use them? AprilTag detection is a little more jittery and the WPILib basic
    algorithms don't do as much validation and determination of the best possible pose. PhotonVision and LimelightVision
    have multiple methods and stable poses on the field are almost always available.

    The roboRIO v1 can do detection and pose calculation but no CPU time left for much of anything else. Limit that usage
    to detection and targeting commands that turn angle to target.

    WPILib + roboRIO v2 (and maybe v1) usage could be a good vision solution for "zero budget" teams to aim at nearby targets.
    For better range some pose validation may need to be added to eliminate some of the jitter. The robot is clamped to the
    floor in AcquireRobotPose (that could be removed). Some teams clamp the pose rotation to the gyro which is not included
    in this implementation but could be added. (Some teams do this for Limelight MT1; MT2 already does this.)

    LimelightVision is a pricey vision solution and the closest to plug-n-play that appeals to many teams.

    PhotonVision software on RPi 4B or other hardware platforms + OV9281 camera sensor + power supply is good vision and moderate cost.
 */

/*
 * Two example commands to align to the 2025 Reefscape reef targets at the #10 AprilTag:
 *   use 3-D pose - activated by XBox controller A button
 *   use turn to point and drive a distance - activated by XBox controller B button
 * Pick the implementation in VisionContainer:
 *   ControllerVision (roboRIO) 2-D & 3-D
 *   PhotonVision 2-D & 3-D
 *   LimelightVision 2-D & 3-D
 */
package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public RobotContainer robotContainer;
    
    public Robot()
    {    
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() // called last in the periodic loop despite always seen written first in Robot.java
    {
        robotContainer.getVisionContainer().updateVision();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}
}
