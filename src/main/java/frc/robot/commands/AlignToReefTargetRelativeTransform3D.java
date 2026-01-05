package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Robot approach target and drive to requested pose based on the relative position to the target.
 * That is, this command drives to a Transform3d coordinate location near the target as opposed to
 * the {@link AlignToReefFieldRelativePose3D} which drives to a field location coordinate.
 * 
 * <p>
 * This example uses chassis speed.
 * 
 * <p>
 * Pick what the drivetrain interface actually needs.
 * 
 * <p>
 * This is an example for any AprilTag in view. Validation of the tag being used must be done
 * and then related to where the target is in relation to the tag. This scheme worked for several
 * targets in ReefScape 2025 because several of the scoring positions where identically relative
 * to their designated tags. Validation had to be done to assure it was one of those in the set of
 * identically relative targets. That tag selection and validation is simplistic in this example.
 * 
 * <p>
 * Thus, in use, this command must have a list of (tags, Transform3d) that are the coordinates of
 * the robot's scoring positions relative to each of the corresponding tags. This example has one
 * tag and two scoring positions - nothing fancy for validation.
 * 
 * <p>
 * Related references:
 * Started with a view of BetaBot2025-develop[ElectroBunny].zip
 * 3 PIDs to align robot to reef tag 10 left or right branches
 * https://www.chiefdelphi.com/t/from-zero-to-auto-align-to-reef-tutorial-how-to-simply-auto-align-to-the-reef/494478
 * 
 * <p>
 * Other references for 3-D poses at LimelightVision.io and photonvision.org
 */
public class AlignToReefTargetRelativeTransform3D extends Command {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private int tagIDDesired;

    // PID constants
    private static final double X_REEF_ALIGNMENT_KP = 1.;
    private static final double Y_REEF_ALIGNMENT_KP = 1.;
    private static final double ROT_REEF_ALIGNMENT_KP = 1.;

    private static final double X_TOLERANCE_REEF_ALIGNMENT = 0.06; // meters
    private static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.06; // meters
    private static final double ROT_TOLERANCE_REEF_ALIGNMENT = Units.degreesToRadians(5.);

    private PIDController xController;
    private PIDController yController;
    private PIDController rotController;

    private static final double DONT_SEE_TAG_WAIT_TIME = 10.; // FIXME that's a long time to drive without knowing where to go; maybe stop motors after .25 sec but keep the command going for 1.?
    // odometry should take over if there is no vision after a short time like 1 iteration? no vision then use odometry?
    private static final double HOLD_POSE_VALIDATION_TIME = 0.3;

    private Timer dontSeeTagTimer;
    private Timer holdPoseValidationTimer;
    private final Transform3d targetBranch; // robot scoring position offset from AprilTag
    private Transform3d target;
    private VisionContainer visionContainer;
    private Drivetrain drivetrain;
    private boolean bail;
    private double xSpeed;
    private double ySpeed;
    private double rotValue;

    public AlignToReefTargetRelativeTransform3D(boolean isRightScore, VisionContainer visionContainer,
            Drivetrain drivetrain) {

        this.visionContainer = visionContainer;
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        // FIXME To score, need a list of the robot's scoring positions relative to the tags

        // Simple example with one tag that has two similar scoring positions so take advantage of that symmetry
        tagIDDesired = 10; // example is short list that isn't even a list

        // need the robot's position to score. Let's use the tag10 + offsets for the targets' pose3d
        // two possible scoring targets to the right and left of desired AprilTag

        if (isRightScore) {
            targetBranch = new Transform3d(1., 0.17, 0., new Rotation3d(0., 0., Math.PI));
        } else {
            targetBranch = new Transform3d(1., -0.17, 0., new Rotation3d(0., 0., Math.PI));
        }

        target = targetBranch;

        xController = new PIDController(X_REEF_ALIGNMENT_KP, 0.0, 0); // +forward/-back translation
        xController.setSetpoint(target.getX());
        xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

        yController = new PIDController(Y_REEF_ALIGNMENT_KP, 0.0, 0); // +left/-right translation
        yController.setSetpoint(target.getY());
        yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

        rotController = new PIDController(ROT_REEF_ALIGNMENT_KP, 0, 0); // +CCW/-CW rotation
        rotController.setSetpoint(target.getRotation().getZ());
        rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);
        rotController.enableContinuousInput(-Math.PI, Math.PI);
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

        if (visionContainer.getRobotPose().isPresent()) { // see a tag so reset countdown to failure timer and process this iteration
            pose = visionContainer.getRobotPose().get();
            dontSeeTagTimer.reset();
        } else { // no tag seen so let the countdown to failure timer run and quit this iteration
            return;
        }

        if (pose.AprilTagId != tagIDDesired) // make sure still looking at the correct tag
        {
            System.out.println("Oops! Looking at wrong tag " + pose.AprilTagId);
            bail = true;
            return;
        }

        if (pose.pose3D.equals(Pose3d.kZero)) // make sure there is 3-D vision; this is a cheat that works most of the time
        {
            System.out.println("Oops! 3-D processing mode not activated in selected vision system");
            bail = true;
            return;
        }

        // Because the robot pose is transform3d from robot to tag, the PID controller must be in reverse
        // Too big robot position means drive forward at +speed to reduce the -delta which is Setpoint-Measurement

        System.out.println("pose error " + (-target.getX() + pose.cameraToTarget.getX()) + " " +
                (-target.getY() + pose.cameraToTarget.getY()) + " " +
                (-target.getRotation().getZ() + pose.cameraToTarget.getRotation().getZ()));

        xSpeed = -xController.calculate(pose.cameraToTarget.getX());
        ySpeed = -yController.calculate(pose.cameraToTarget.getY());
        rotValue = -rotController.calculate(pose.cameraToTarget.getRotation().getZ());
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

        if (interrupted) {
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
        } else {
            drivetrain.driveRobotRelative(new ChassisSpeeds(xSpeed, ySpeed, rotValue));
            holdPoseValidationTimer.reset();
        }

        var dontSeeTag = dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME);
        var holdPose = holdPoseValidationTimer.hasElapsed(HOLD_POSE_VALIDATION_TIME);

        if (bail) {
            System.out.print(" ended by bail ");
        }

        if (dontSeeTag) {
            System.out.print(" ended by don't see tag ");
        }

        if (holdPose) {
            System.out.print(" ended by correct pose held ");
        }

        return bail || dontSeeTag || holdPose;
    }
}
