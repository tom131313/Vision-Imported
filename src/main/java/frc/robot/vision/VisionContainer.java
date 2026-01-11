package frc.robot.vision;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.utils.Network.getMyIPAddress;

import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import static frc.robot.Config.VisionSettings;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AlignToReefFieldRelativePose3D;
import frc.robot.commands.AlignToReefTagRelativeArcade2D;
import frc.robot.commands.AlignToReefTargetRelativeTransform3D;

/**
 * This class is a wrapper for the three vision classes {@link ControllerVision}, {@link
 * PhotonVision}, and {@link LimelightVision}
 * <p>
 * ControllerVision (roboRIO) returns team specified robot pose based on the tag ID that is closest
 * to the robot (this needs work; that might not always be appropriate).
 * <p>
 * PhotonVision returns its best pose (but not the very best pose estimator that includes odometry
 * and gyro that teams should add).
 * <p>
 * LimelightVision returns team specified filtered selection from MegaTag or MegaTag2.
 * (This is a reasonable default but could use more tuning. See {@link LimelightVision#update()})
 * <p>
 * After selecting and configuring the desired vision system {@link Config}, use the
 * robot pose as in example commands {@link AlignToReefFieldRelativePose3D},
 * {@link AlignToReefTargetRelativeTransform3D} and {@link AlignToReefTagRelativeArcade2D}
 * <p>
 * Getters are available for each of the three vision systems. This allows use of other low level
 * methods that may be available within the systems' classes. That also helps reveal the following
 * getters which are duplicative with the RobotPose class data so there is no reason to use them.
 * <p>
 * Each vision system extends {@link CameraBase} which provides for
 * 
 * <pre>
 * <code>
 *public abstract void update();
 *public abstract boolean isFresh();
 *public abstract int getTagID();
 *public abstract Pose3d getPose3d();
 *public abstract Pose2d getPose2d();
 *public abstract double getTX();
 *public abstract double getTY();
 *public abstract Transform3d getCameraToTarget();
 * </code>
 * </pre>
 * <p>
 * Usage of these methods requires that after "update()" the "isFresh()" is checked before using
 * any of the getters.
 * <p>
 * Since all of these data are available in the RobotPose there isn't a reason to use these methods
 * in this example project. Other projects may use them.
 */
public class VisionContainer {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());

        System.loadLibrary(org.opencv.core.Core.NATIVE_LIBRARY_NAME); // OpenCV required for all Vision Processes

        try { // likely needed for 3-D pose usage
            Class.forName(frc.robot.vision.AprilTagsLocations.class.getCanonicalName());
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
    }

    public enum VisionSelector {
        useControllerVision, usePhotonVision, useLimelightVision
    }

    private VisionSelector visionSelector = VisionSettings.visionSelector;

    private ControllerVision controllerVision = null; // only for Controller (roboRIO)
    private PhotonVision photonVision = null; // only for PV
    private LimelightVision limelightVision = null; // only for LL
    private AtomicReference<Optional<RobotPose>> robotPose =
        new AtomicReference<Optional<RobotPose>>(Optional.empty());
    private Transform3d robotToCamera;
    private boolean vision = false; // initially not activated

    public VisionContainer() {

        DriverStation.reportWarning("vision selection " + visionSelector, false);

        switch (visionSelector) {
            case useControllerVision:
                var cameraDeviceId = VisionSettings.cameraDeviceId;
                if (ControllerVision.isAvailable(cameraDeviceId)) {
                    var robotToCameraCV = new Transform3d(new Translation3d(0.5, 0.0, 0.25),
                            new Rotation3d(0, Units.degreesToRadians(-25.), 0));
                    robotToCamera = robotToCameraCV;
                    controllerVision = new ControllerVision(cameraDeviceId,
                            VisionSettings.camera, VisionSettings.usePose3D, robotToCameraCV);

                    // vision runs as a "background" thread to minimize interference with the other
                    // robot actions. This may increase latency. Priority could be experimented with.
                    Thread acquireControllerCamera = new Thread(controllerVision);
                    acquireControllerCamera.setName("roboRIOCameraPose");
                    acquireControllerCamera.setPriority(acquireControllerCamera.getPriority() - 1);
                    acquireControllerCamera.setDaemon(true);
                    acquireControllerCamera.start();
                    vision = true;
                } else {
                    DriverStation.reportWarning("No ControllerVision process", false);
                    System.out.println("Make sure camera is plugged in" +
                            (Robot.isSimulation() ? " and number is correct (try the other one)" : ""));

                    controllerVision = null;
                }

                break;

            case usePhotonVision:
                var cameraNamePV = VisionSettings.cameraNamePV;
                var robotToCameraPV = VisionSettings.robotToCameraPV;
                robotToCamera = robotToCameraPV;

                // PV started connecting when the NT server started but still it's a race to make
                // a connection before this instantiation and check
                try {
                    Thread.sleep(3000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                photonVision = new PhotonVision(cameraNamePV, robotToCameraPV);

                final var retryDelay = Seconds.of(1);
                final var retryLimit = 20;

                checkConnection: {
                    for (int i = 1; i <= retryLimit; i++) {
                        if (photonVision.camera.isConnected()) {
                            vision = true;
                            break checkConnection;
                        }
                        System.out.println("attempt " + i + " of " + retryLimit + " to attach to PhotonVision named "
                                + cameraNamePV);
                        try {
                            Thread.sleep((long) retryDelay.in(Milliseconds));
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                    photonVision.camera.getAllUnreadResults(); // force PV to issue no camera by that name but there are other PV cameras

                    if (Robot.isSimulation()) {
                        System.out.println(
                                "Enter in the PV Settings Team Number/NetworkTables Server Address the IP address of this simulation device on the interface to the PhotonVision device");
                        try {
                            System.out.println(
                                    "If PV running on the same device as this simulation, then use any of the IP addresses assign to that device such as 127.0.0.1, localhost, or "
                                            +
                                            InetAddress.getLocalHost().getHostAddress());
                        } catch (UnknownHostException e) {
                            e.printStackTrace();
                        }
                        System.out.println(getMyIPAddress());
                    } else // it's real roboRIO
                    {
                        System.out.println("Make sure the team number " + RobotController.getTeamNumber()
                                + " is entered to attach to NetworkTables");
                    }

                    DriverStation.reportWarning("No PhotonVision connection", false);

                    photonVision = null; // put it back to null since constructor didn't really make a viable camera
                }

                break;

            case useLimelightVision:
                /*
                 * Add additional Limelight setups as indicated below
                 */
                var limelightName = VisionSettings.limelightName;
                if (LimelightVision.isAvailable(limelightName)) {
                    limelightVision = new LimelightVision(limelightName, RobotContainer.getDrivetrain());
                    var c = LimelightHelpers.getCameraPose3d_RobotSpace(limelightName); // retrieve camera position in LL
                    var robotToCameraLL = new Transform3d(c.getTranslation(),
                            new Rotation3d(c.getRotation().getX(), -c.getRotation().getY(), c.getRotation().getZ())); // - pitch to match CV & PV
                    robotToCamera = robotToCameraLL;
                    vision = true;
                    // put additional LL setup as needed
                    // LimelightHelpers.setStreamMode_PiPSecondary(limelightName); // if there is a driver camera, where to put the image - pick your favorite
                } else {
                    DriverStation.reportWarning("No LimelightVision connection", false);

                    limelightVision = null;
                }

                break;

            default:
                DriverStation.reportWarning("Invalid vision system option selected in VisionContainer constructor, got: "
                        + visionSelector.toString(),
                        true);
                break;
        }
    }

    /**
     * 
     * @return if there is a Vision System active
     */
    public boolean vision() {
        return vision;
    }

    /**
     * periodically executed to update vision information
     */
    public void update() {
        if (!vision()) {
            return;
        }

        switch (visionSelector) {
            case useControllerVision:
                controllerVision.update();
                if (controllerVision.isFresh()) {
                    robotPose.set(Optional.of(new RobotPose(
                            controllerVision.getTagID(),
                            controllerVision.getTX(), controllerVision.getTY(),
                            controllerVision.getPose3d(),
                            controllerVision.getCameraToTarget())));
                } else {
                    robotPose.set(Optional.empty());
                }

                break;

            case usePhotonVision:
                photonVision.update();
                if (photonVision.isFresh()) {
                    robotPose.set(Optional.of(new RobotPose(
                            photonVision.getTagID(),
                            photonVision.getTX(), photonVision.getTY(),
                            photonVision.getPose3d(),
                            photonVision.getCameraToTarget())));
                    // System.out.println("PV " + photonVision.getTagID() + ", " + photonVision.getTX() + ", " + photonVision.getTY() + ", " + photonVision.getPose3d());
                } else {
                    robotPose.set(Optional.empty());
                }

                break;

            case useLimelightVision:
                limelightVision.update();
                if (limelightVision.isFresh()) {
                    robotPose.set(Optional.of(new RobotPose(
                            limelightVision.getTagID(),
                            limelightVision.getTX(), limelightVision.getTY(),
                            limelightVision.getPose3d(),
                            limelightVision.getCameraToTarget())));
                    // System.out.println((limelightVision.getSuggestResetOdometry() ?
                    // "reset odometry pose " : "addVisionMeasurement pose ") +
                    // limelightVision.getPose2d() + (limelightVision.isMegaTag2() ? " MegaTag2 pose" : " MegaTag pose"));
                } else {
                    robotPose.set(Optional.empty());
                }

                break;

            default:
                DriverStation.reportWarning("Invalid vision system option selected in VisionContainer update, got: "
                        + visionSelector.toString(),
                        true);
                break;
        }

        // getRobotPose().ifPresentOrElse
        // (
        // (pose)-> {
        // var t = getRobotPose().get().cameraToTarget;
        // System.out.println("tag is forward " + t.getX() + "m, left " + t.getY() + "m, up " + t.getZ() +
        // "m, roll " + Units.radiansToDegrees(t.getRotation().getX()) +
        // "deg, pitch " + Units.radiansToDegrees(t.getRotation().getY()) +
        // "deg, yaw " + Units.radiansToDegrees(t.getRotation().getZ()) + "deg");},
        // ()-> System.out.print("X")
        // );
    }

    /**
     * Robot pose for whomever wants it
     * 
     * @return RobotPose
     */
    public Optional<RobotPose> getRobotPose() {
        return robotPose.get();
    }

    /**
     * 
     * @return location of the camera wrt the robot
     */
    public Transform3d getRobotToCamera() {
        return robotToCamera;
    }

    /**
     * Getter for vision system for CameraBase getters
     * 
     * @return
     */
    public PhotonVision getPhotonVision() {
        return photonVision;
    }

    /**
     * Getter for vision system for CameraBase getters
     * 
     * @return
     */
    public LimelightVision getLimelightVision() {
        return limelightVision;
    }

    /**
     * Getter for vision system for CameraBase getters
     * 
     * @return
     */
    public ControllerVision getControllerVision() {
        return controllerVision;
    }
}
