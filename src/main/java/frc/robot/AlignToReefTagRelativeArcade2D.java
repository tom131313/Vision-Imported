package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Robot approach target to desired distance and turn to desired yaw.
 * 
 * <p>This example shows both differential drive arcade speed and chassis speed.
 * 
 * <p>The units for those two concepts are very different so the PID Kp constants
 * must be very different but they aren't in this example. Don't do both ways -
 * pick what the drivetrain interface needs.
 * 
 * <p>This is an example for only one AprilTag and that is determined in the code.
 * 
 * <p>Related references:
 * <p>https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-aiming-with-visual-servoing
 * <p>https://docs.photonvision.org/en/latest/docs/examples/aimingatatarget.html
 * <p>https://docs.photonvision.org/en/latest/docs/examples/aimandrange.html
 */
public class AlignToReefTagRelativeArcade2D extends Command {
    static
    {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // need to know what target is seen to know target height from floor and where scoring is relative to it
    private int tagIDDesired = 10; // FIXME need dynamic determination of the tag to use
    private double targetHeight = AprilTagsLocations.getTagLocation(tagIDDesired).getZ();
    private double scoringDistance = 0.6; // parallel to the floor = meters
    private double scoringAngle = Units.degreesToRadians(5.); // slightly off of tag - radians

    private double cameraHeight;
    private double cameraPitch;

    private final VisionContainer visionContainer;
    // essentially fake drive motors to show how it might be done
    private final DifferentialDrive robotDrive;
    private final PWMTalonFX leftMotor = new PWMTalonFX(0);
    private final PWMTalonFX rightMotor = new PWMTalonFX(1);

  // PID constants
  public static final double DISTANCE_REEF_ALIGNMENT_KP = 1.;
  public static final double ROT_REEF_ALIGNMENT_KP = 1.;

  public static final double DISTANCE_TOLERANCE_REEF_ALIGNMENT = 0.06; // meters
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = Units.degreesToRadians(3.);

  private PIDController distanceController;
  private PIDController rotController;

  public static final double DONT_SEE_TAG_WAIT_TIME = 10.; //FIXME that's a long time to drive without knowing where to go; maybe stop motors after .25 sec but keep the command going for 1.?
  // odometry should take over if there is no vision after a short time like 1 iteration? no vision then use odometry?
  // this example does not include odometry

  public static final double HOLD_POSE_VALIDATION_TIME = 0.3;

  private Timer dontSeeTagTimer;
  private Timer holdPoseValidationTimer;

  private boolean bail;
  private double distanceSpeed;
  private double rotSpeed;

    public AlignToReefTagRelativeArcade2D(VisionContainer visionContainer)
    {
        //FIXME addRequirements(drivetrain);
        this.visionContainer = visionContainer;
        cameraHeight = visionContainer.getRobotToCamera().getZ();
        cameraPitch = visionContainer.getRobotToCamera().getRotation().getY();
        robotDrive = new DifferentialDrive((speed)->leftMotor.set(speed), (speed)->rightMotor.set(speed));
        robotDrive.setSafetyEnabled(false);

        // System.out.println("info " + target + " " + AprilTagsLocations.getTagLocation(10) + " " + targetBranch + " end info");
    
        distanceController = new PIDController(DISTANCE_REEF_ALIGNMENT_KP, 0.0, 0); // +forward/-back translation
        distanceController.setSetpoint(scoringDistance);
        distanceController.setTolerance(DISTANCE_TOLERANCE_REEF_ALIGNMENT);
     
        rotController = new PIDController(ROT_REEF_ALIGNMENT_KP, 0, 0); // +CCW/-CW rotation
        rotController.setSetpoint(scoringAngle);
        rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);
        rotController.enableContinuousInput(0., Math.PI*2.);
    }

    public void initialize()
    {
        holdPoseValidationTimer = new Timer();
        holdPoseValidationTimer.start();
        dontSeeTagTimer = new Timer();
        dontSeeTagTimer.start();
        bail = false; 
        distanceController.reset();
        rotController.reset();
      }
    
    public void execute()
    {
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
  
        // get the camera frame pose information
        var targetPitch = pose.pitch;
        var targetYaw = pose.yaw;
  
        var distanceToTarget = distanceToTarget(cameraHeight, targetHeight, cameraPitch, Units.degreesToRadians(targetPitch));

        distanceSpeed = distanceController.calculate(distanceToTarget);
        rotSpeed = rotController.calculate(targetYaw);
        // This rotation calculate() uses the camera which generally is slow to respond
        // (limelight claims to the contrary not withstanding).
        // Generally it's superior to use the PID controller on the gyro reading. The PID
        // setpoint would be the gyro offset from the vision heading. Although the
        // desired gyro heading won't change much, update that setpoint as new vision data
        // is available.

        System.out.println("pitch " + targetPitch + ", yaw " + targetYaw + ", distance " + distanceToTarget + ", rot speed " + rotSpeed + ", distance speed " + distanceSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Two examples of how to drive - pick one way - maybe not either but what your drivetrain requires
        robotDrive.arcadeDrive(0, 0, false);
        driveRobotRelative(new ChassisSpeeds(0, 0, 0));

        if (interrupted)
        {
            System.out.println("ended by interrupted");
        }
    }

    public boolean isFinished()
    {
        if (rotController.atSetpoint() &&
            distanceController.atSetpoint()) {
            // at goal pose so stop and see if it settles
            // Two examples of how to drive - pick one way - maybe not either but what your drivetrain requires
            robotDrive.arcadeDrive(0, 0, false);
            driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        }
        else {
            // Two examples of how to drive - pick one way - maybe not either but what your drivetrain requires
            robotDrive.arcadeDrive(distanceSpeed, rotSpeed, false);
            driveRobotRelative(new ChassisSpeeds(distanceSpeed, 0., rotSpeed));
            holdPoseValidationTimer.reset();
        }

        var dontSeeTag = dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME);
        var holdPose = holdPoseValidationTimer.hasElapsed(HOLD_POSE_VALIDATION_TIME);

        if (bail)
        {
          System.out.print(" ended by bail " +  bail);
        }
    
        if (dontSeeTag)
        {
          System.out.print(" ended by don't see tag " + dontSeeTag);
        }
    
        if (holdPose)
        {
          System.out.print(" ended by correct pose held " + holdPose);
        }
    
        return bail || dontSeeTag || holdPose;
    }

    /**
     * Estimation of shortest distance parallel to the floor from camera to the vertical wall that a
     * target is mounted on based on Right Triangle geometry.
     * <p>See https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
     * 
     * <p>Note that the camera must mounted with 0 roll (rotation around the forward axis).
     * 
     * <p>  Highest accuracy if the camera is in line side-to-side with target and large difference in height.
     * That's inconsistent with camera positions that are best for 3-D pose estimation where the camera
     * should be at least a small angle to the side of a target in addition to being at different heights.
     * 
     * <p>There must be significant height difference for this to work accurately. Slight jitter in the
     * pitch causes a lot of jitter in the distance calculation (tangent function). If it doesn't work,
     * then try a lookup table of hand measured values but that still suffers exactly the same problem.
     * Filtering of the distance signal such as Savitzky-Golay least squares filtering might help (that's
     * unconfirmed speculation).
     * 
     * A better distance measurement would come from a proper distance sensor such as the analog mode
     * (at least for 2026) of https://swyftrobotics.com/products/swyft-ranger-distance-sensor
     * 
     * A distance sensor may respond more quickly than a camera - a lot of variation in the speed of
     * cameras and distance sensors. The SWYFT is 2.5 Hz to 15 Hz which is much slower than or about
     * as fast as cameras. PhotonVision and LimelightVision may be substantially faster than this.
     *
     * @param cameraHeight height of the camera off the floor in meters.
     * @param targetHeight height of the target off the floor in meters.
     * @param cameraPitch pitch of the camera from the horizontal plane in radians.
     *     Positive values are up.
     * @param targetPitch pitch of the target to the camera's lens in radians. Positive
     *     values are up.
     * @return estimated ground distance from the camera to the target in meters.
     */
    public static double distanceToTarget(
            double cameraHeight, // fixed
            double targetHeight, // fixed
            double cameraPitch,  // fixed
            double targetPitch)  // measured each frame
    {
        return (targetHeight - cameraHeight) // distance between heights
               / Math.tan(cameraPitch + targetPitch); // total angle with the floor
    }

    // // look-up-table alternative to distanceToTarget() may be slightly more accurate but much less flexible and much harder to use.
    // // It still suffers if the camera and target are at about the same height.
    // distanceToTarget = new InterpolatingDoubleTreeMap();
    // // example degrees pitch to meters distance
    // // make your measurements to put enough points for linear interpolation to be accurate
    // distanceToTarget.put(-6., 1.5);
    // distanceToTarget.put(0., 0.5);
    // distanceToTarget.put(9., 0.25);

    // // SWYFT analog distance sensor
    // AnalogInput distanceSensor = new AnalogInput(0);
    // distanceToTarget =  distanceSensor.getVoltage()*32.50930976)-2.695384202);


    /** Simulate (extremely badly) the drivetrain.driveRobotRelative to test this                
     * Robot Centric Orientation (not Field centric)
     * <p> one of two examples of how to drive - pick one way - maybe not either but
     * what your drivetrain requires
     * @param chassisSpeeds the desired robot chassis speed
     * @return drive command
     */
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds)
    {
      // goal current speeds
      //  System.out.print(" " + chassisSpeeds + " ");
    }
}
