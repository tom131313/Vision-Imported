package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Robot approach target and drive to requested pose based on the selected target.
 * 
 * <p>This example uses chassis speed.
 * 
 * <p>Pick what the drivetrain interface actually needs. A dummy CTRE swerve is
 * included to allow the PathPlanner drive-to-target to compile.
 * 
 * <p>This is an example for only one AprilTag and that is determined in the code.
 * 
 * <p>Related references:
 * Started with a view of BetaBot2025-develop[ElectroBunny].zip
 *    3 PIDs to align robot to reef tag 10 left or right branches
 *    https://www.chiefdelphi.com/t/from-zero-to-auto-align-to-reef-tutorial-how-to-simply-auto-align-to-the-reef/494478
 * 
 * <p>Other references for 3-D poses at LimelightVision.io and photonvision.org
 */
public class AlignToReefFieldRelativePose3D extends Command {
  static
  {
    System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
  }
  
  private int tagIDDesired = 10; // FIXME need dynamic determination of the tag to use

  // PID constants
  public static final double X_REEF_ALIGNMENT_KP = 1.;
  public static final double Y_REEF_ALIGNMENT_KP = 1.;
  public static final double ROT_REEF_ALIGNMENT_KP = 1.;

  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.06; // meters
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.06; // meters
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = Units.degreesToRadians(3.);

  private PIDController xController;
  private PIDController yController;
  private PIDController rotController;

  public static final double DONT_SEE_TAG_WAIT_TIME = 10.; //FIXME that's a long time to drive without knowing where to go; maybe stop motors after .25 sec but keep the command going for 1.?
  // odometry should take over if there is no vision after a short time like 1 iteration? no vision then use odometry?
  public static final double HOLD_POSE_VALIDATION_TIME = 0.3;

  private Timer dontSeeTagTimer;
  private Timer holdPoseValidationTimer;
  private final Transform3d targetBranch; // offset from AprilTag in field pose
  Pose3d target;
  private VisionContainer visionContainer;
  private boolean bail;
  double xSpeed;
  double ySpeed;
  double rotValue;

  public AlignToReefFieldRelativePose3D(boolean isRightScore, VisionContainer visionContainer) {

    //FIXME addRequirements(drivetrain);

    this.visionContainer = visionContainer;
  
    // need the robot's position to score. Let's use the tag10 + offsets for the targets' pose3d
    // two possible scoring targets to the right and left of desired AprilTag
    if (isRightScore)
    { // location of the robot to score relative to the tag in field coordinates
        targetBranch = new Transform3d(1., 0.17, 0., new Rotation3d(0., 0., Math.PI));
    }
    else
    {
        targetBranch = new Transform3d(1., -0.17, 0., new Rotation3d(0., 0., Math.PI));
    }

    target = AprilTagsLocations.getTagLocation(10).plus(targetBranch);
    // System.out.println("info " + target + " " + AprilTagsLocations.getTagLocation(10) + " " + targetBranch + " end info");

    xController = new PIDController(X_REEF_ALIGNMENT_KP, 0.0, 0); // +forward/-back translation
    xController.setSetpoint(target.getX());
    xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

    yController = new PIDController(Y_REEF_ALIGNMENT_KP, 0.0, 0); // +left/-right translation
    yController.setSetpoint(target.getY());
    yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

    rotController = new PIDController(ROT_REEF_ALIGNMENT_KP, 0, 0); // +CCW/-CW rotation
    rotController.setSetpoint(target.getRotation().getZ());
    rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);
    rotController.enableContinuousInput(0., Math.PI*2.); //FIXME change to the gyro range
  }

  @Override
  public void initialize() {

    holdPoseValidationTimer = new Timer();
    holdPoseValidationTimer.start();
    dontSeeTagTimer = new Timer();
    dontSeeTagTimer.start();
    bail = false;
    xController.reset();
    yController.reset();
    rotController.reset();
  }

  @Override
  public void execute() {

    RobotPose pose;

    if (visionContainer.getRobotPose().isPresent())
    { // see a tag so reset countdown to failure timer and process this iteration
      pose = visionContainer.getRobotPose().get();
      dontSeeTagTimer.reset();      
    }
    else
    { // no tag seen so let the countdown to failure timer run and quit this iteration
      return;
    }

    if (pose.AprilTagId != tagIDDesired) // make sure still looking at the correct tag
    {
      System.out.println("Oops! Looking at wrong tag");
      bail = true;
      return;
    }    

    if (pose.pose3D.equals(Pose3d.kZero)) // make sure there is 3-D vision
    {
      System.out.println("Oops! 3-D processing mode not activated in selected vision system");
      bail = true;
      return;
    }

    System.out.println("pose error " + (target.getX()-pose.pose3D.getX()) + " " + (target.getY()-pose.pose3D.getY()) + " "  + (target.getRotation().getZ()-pose.pose3D.getRotation().getZ()));

    xSpeed = xController.calculate(pose.pose3D.getX());
    ySpeed = yController.calculate(pose.pose3D.getY());
    rotValue = rotController.calculate(pose.pose3D.getRotation().getZ());
    // These pose calculate() use the camera which generally is slow to respond
    // (limelight claims to the contrary not withstanding).
    // Generally it's superior to use the robot pose as the best estimate using
    // odometry, gyro, and vision.
    // Also, odometry would be available if vision wasn't available because of slow
    // updates or no longer in view.
   }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));

    if (interrupted)
    {
        System.out.println(" ended by interrupted ");
    }
  }

  @Override
  public boolean isFinished() {

    if (rotController.atSetpoint() &&
        yController.atSetpoint() &&
        xController.atSetpoint()) {
        // at goal pose so stop and see if it settles
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    }
    else {
      drivetrain.driveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, rotValue));
      holdPoseValidationTimer.reset();
    }

    var dontSeeTag = dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME);
    var holdPose = holdPoseValidationTimer.hasElapsed(HOLD_POSE_VALIDATION_TIME);

    if (bail)
    {
      System.out.print(" ended by bail ");
    }

    if (dontSeeTag)
    {
      System.out.print(" ended by don't see tag ");
    }

    if (holdPose)
    {
      System.out.print(" ended by correct pose held ");
    }

    return bail || dontSeeTag || holdPose;
  }

  /*******************************************************************************************
   * 
   * dummy Drivetrain and SwerveDriveState classes to allow this sample PathPlanner to compile
   */

    private Drivetrain drivetrain = this.new Drivetrain();

    /**
     * Sample of PathPlanner followPath to drive to target position. It is essentially the PID
     * controllers similar to the above command controllers and would replace the above command.
     * <p>Drives autonomously from the given pose to the target pose using PathPlanner
     * <p>PathPlanner AutoBuilder must be configured and tuned before usage.
     * <p>This code has not been verified but came from functioning team code.
     * @param targetPose
     * @param currentPose
     * @return Command to follow path on the fly
     * @author Biggie Cheese
     */
    public /*static*/ Command driveToPositionCommand(Pose2d targetPose, Pose2d currentPose)
    {
        PathConstraints constraints = new PathConstraints(2.0, 1.0, Units.degreesToRadians(360), Units.degreesToRadians(360));
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                                    new Pose2d(currentPose.getTranslation(), currentPose.getRotation()),
                                    new Pose2d(targetPose.getTranslation(), targetPose.getRotation()));          

        double vxMetersPerSecond = drivetrain.getState().Speeds.vxMetersPerSecond;
        double vyMetersPerSecond = drivetrain.getState().Speeds.vyMetersPerSecond;

        double velocity = Math.hypot(vxMetersPerSecond, vyMetersPerSecond);

        Rotation2d rotation = drivetrain.getPose().getRotation();

        IdealStartingState idealStartingState = new IdealStartingState(velocity, rotation);

        PathPlannerPath path = new PathPlannerPath(
                                    waypoints,
                                    constraints,
                                    idealStartingState, // set this to null if not working
                                    new GoalEndState(0.0, targetPose.getRotation()));
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    private class Drivetrain
    {
        /** Stub to simulate (extremely badly - nothing to it) for testing.
         * Robot Centric Orientation (not Field centric)
         * @param chassisSpeeds the desired robot chassis speed
         * @return drive command
         */  
        public void driveRobotRelative(ChassisSpeeds chassisSpeeds)
        {
          // goal current speeds
          // System.out.print(" " + chassisSpeeds + " ");
        }

        /**
         * dummy
         */
        private Pose2d getPose()
        {
            return getState().pose;
        }

        /**
         * dummy
         */
        private SwerveDriveState getState()
        {
            return new SwerveDriveState();          
        }

        /**
         * dummy
         */
        private class SwerveDriveState
        {
            /** The current pose of the robot */
            public Pose2d pose = new Pose2d();
            /** The current robot-centric velocity */
            public ChassisSpeeds Speeds = new ChassisSpeeds();
        }
   }
}
