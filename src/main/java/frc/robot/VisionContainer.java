package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is a wrapper for the three vision classes {@link ControllerVision}, {@link PhotonVision}, and {@link LimelightVision}
 * <p>ControllerVision (roboRIO) returns team specified pose of lowest tag ID
 *  (This probably needs work! See {@link #periodic()} ControllerVision block)
 * <p>PhotonVision returns its best pose
 * <p>LimelightVision returns team specified filtered selection from MegaTag or MegaTag2
 *  (This is a reasonable default but could use work! See {@link LimelightVision#update()})
 * 
 * After selecting and configuring the desired vision system, use the robot pose as in example commands
 * {@link AlignToReefFieldRelativePose3D} and {@link AlignToReefTagRelativeArcade2D}
 * briefly:
 *  <pre><code>
    RobotPose pose;
    double cameraHeight
    if (visionContainer.getRobotPose().isPresent())
    { // see a tag
      pose = visionContainer.getRobotPose().get();
      // some distance calculations require camera height from floor
      cameraHeight = visionContainer.getRobotToCamera().getZ();  
    }
    else
    { // no tag seen
      return;
    }
    // RobotPose is the AprilTagId associated with the robot yaw and pitch wrt the tag and Pose3d wrt the field
</pre></code>

 * Getters are available for each of the three vision systems. This allows use of other low level methods that may be available within the systems' classes.
 * That also helps reveal the following getters which are duplicative with the RobotPose class data so there is no reason to use them.
 * Each vision system extends CameraBase which provides for 
 *   public abstract void update();
 *   public abstract boolean isFresh();
 *   public abstract int getTagID();
 *   public abstract Pose3d getPose3d();
 *   public abstract Pose2d getPose2d();
 *   public abstract double getTX();
 *   public abstract double getTY();
 * 
 * Usage of these methods requires that after "update()" the "isFresh()" is checked before using any of the getters.
 * Since all of these data are available in the RobotPose there shouldn't be a reason to use these methods.
 * Maybe they should be protected or unspecified instead of public.
 */
public class VisionContainer extends SubsystemBase {
  static {
    System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName()); // optional show class has started

    System.loadLibrary(org.opencv.core.Core.NATIVE_LIBRARY_NAME); // OpenCV required for all Vision Processes

    try { // likely needed for 3-D pose usage
      Class.forName("frc.robot.AprilTagsLocations");
    } catch (ClassNotFoundException e) {
      e.printStackTrace();
    }
  }

  public enum VisionSelector{useControllerVision, usePhotonVision, useLimelightVision}
  private VisionSelector visionSelector;

  private ControllerVision controllerVision = null; // only for Controller (roboRIO)
  private PhotonVision photonVision = null; // only for PV
  private LimelightVision limelightVision = null; // only for LL

  private AtomicReference<Optional<RobotPose>> robotPose = new AtomicReference<Optional<RobotPose>>(Optional.empty());
  private Transform3d robotToCamera;
  private boolean vision = false;

  public VisionContainer(VisionSelector visionSelector)
  {
    this.visionSelector = visionSelector;

    switch (visionSelector)
    {
    case useControllerVision:
      // Select the camera model and resolution from the list of cameras
      // that have calibration data.

      // Select if the Pose3D calculation is to be performed.
      
      // Enter the 3-D translation (location) of your camera wrt the bottom, center of the robot.
 
      // Caution: roboRIO v1 has constrained memory and cpu.

      // For v1 use camera resolution about 320x240 maybe a little higher.
      // Experiment with your team usage.

      // "usePose3D" determines if the time consuming calculation for 3-D pose is performed.
      // If this setting is "false", then only the pitch and yaw of the robot wrt the AprilTag is
      // available and can they can be used for what LimeLight describes as "servoing" mode.
      // roboRIO v1 can do Pose3D but little time is left for other processes.
      // v1 memory limits AprilTag bitsCorrected to 1 error bit (up to 2 on roboRIO v2)

      // Select the camera device id

      // Enter the camera location wrt the robot. Example camera mounted facing
      // forward, half a meter forward of center, quarter a meter up from center
      // pointing up 25 degrees.

      // If in Simulation mode on a Windows PC:

      //   If the external USB camera is plugged in at (before) boot-up time,
      //   then it is cameraDeviceId = 0 after booting.

      //   If the PC has an internal camera (common with laptops) and the
      //   external camera is plugged in after boot-up, then the cameraDeviceId = 1.
      var cameraDeviceId = 0;
      if (ControllerVision.isAvailable(cameraDeviceId))
      {
        boolean usePose3D = true;
        var robotToCameraCV = new Transform3d(new Translation3d(0.5, 0.0, 0.25),
                  new Rotation3d(0,Units.degreesToRadians(-25.),0));
        robotToCamera = robotToCameraCV;
        controllerVision = new ControllerVision(cameraDeviceId, ControllerVision.CameraOption.ArduCam320x240, usePose3D, robotToCameraCV);

        // vision runs as a "background" thread to minimize interference with the other
        // robot actions. This may increase latency. Priority could be experimented with.
        Thread acquireControllerCamera = new Thread(controllerVision);
        acquireControllerCamera.setName("roboRIOCameraPose");
        acquireControllerCamera.setPriority(acquireControllerCamera.getPriority() - 1);
        acquireControllerCamera.setDaemon(true);
        acquireControllerCamera.start();
        vision = true;
      }
      else
      {
        DriverStation.reportWarning("No ControllerVision process", false);
        controllerVision = null;
      }

      break;

    case usePhotonVision:
      // Select the camera name

      // Enter the camera location wrt the robot. Example camera mounted facing
      // forward, half a meter forward of center, quarter a meter up from center
      // pointing up 25 degrees.

      var cameraName = "Microsoft_LifeCam_HD-3000";
      var robotToCameraPV = new Transform3d(new Translation3d(0.5, 0.0, 0.25),
                new Rotation3d(0,Units.degreesToRadians(-25.),0));
      robotToCamera = robotToCameraPV;

      // PV started connecting when the NT server started but still it's a race to make
      // a connection before this instantiation and check
      try {
        Thread.sleep(3000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }

      photonVision = new PhotonVision(cameraName, robotToCameraPV);

      final var retryDelay = Seconds.of(1);
      final var retryLimit = 20;
      
      checkConnection:
      {
      for (int i = 1; i <= retryLimit; i++)
      {
        if (photonVision.camera.isConnected())
        {
          vision = true;
          break checkConnection;
        }
        System.out.println("attempt " + i + " of " + retryLimit + " to attach to PhotonVision named " + cameraName);      
        try {
          Thread.sleep((long)retryDelay.in(Milliseconds));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
      DriverStation.reportWarning("No PhotonVision connection", false);
      photonVision.camera.getAllUnreadResults(); // force PV to issue no camera by that name but there are other PV cameras
      photonVision = null; // put it back to null since constructor didn't really make a viable camera
      }

      break;

    case useLimelightVision:
      /*
      * Caution: Limelight pitch setup has the opposite sign of PhotonVision and ControllerVision
      * LL positive pitch is pointing up
      * Snap Robot to Floor is only for MegaTag; MegaTag2 always snaps to floor.
      * Further than 1.1m MegaTag tends to have a lot of jitter especially in the z (height) axis.
      */
      // Select the limelight name
      var limelightName = "limelight";
      if (LimelightVision.isAvailable(limelightName))
      {
          limelightVision = new LimelightVision(limelightName);
          var c = LimelightHelpers.getCameraPose3d_RobotSpace(limelightName); // retrieve camera position in LL
          var robotToCameraLL = new Transform3d(
                c.getX(), c.getY(), c.getZ(),
                new Rotation3d(c.getRotation().getX(), c.getRotation().getY(), c.getRotation().getZ())); 
          robotToCamera = robotToCameraLL;
          vision = true;
          // put other LL setup as needed
          // LimelightHelpers.setStreamMode_PiPSecondary(limelightName); // if there is a driver camera, where to put the image - pick your favorite
      }
      else
      {
          DriverStation.reportWarning("No LimelightVision connection", false);
          limelightVision = null;
      }

     break;
    
    default:
        System.out.println("impossible unless the enum is wrong");
        break;
    }
  }

  /**
   * 
   * @return if there is a Vision System active
   */
  public boolean vision()
  {
    return vision;
  }

  /**
   * periodically executed via Subsystem
   */
  public void periodic()
  {
    if (! vision())
    {
      return;
    }

    switch (visionSelector)
    {
    case useControllerVision:
        controllerVision.update();
        if (controllerVision.isFresh())
        {
          if (controllerVision.getPoses().size() >= 1)
          {
            //FIXME arbitrarily pick lowest tag id; this needs to be thought through
            robotPose.set(Optional.of(controllerVision.getPoses().get(0).clone()));
            // controllerVision.getPoses().forEach(pose -> System.out.println("CV " + pose)); // show all available data
          }
        }
        else
        {
          robotPose.set(Optional.empty());
        }

        break;

    case usePhotonVision:
        photonVision.update();
        if (photonVision.isFresh())
        {
          robotPose.set(Optional.of(new RobotPose(photonVision.getTagID(), photonVision.getTX(), photonVision.getTY(), photonVision.getPose3d())));
          // System.out.println("PV " + photonVision.getTagID() + ", " + photonVision.getTX() + ", " + photonVision.getTY() + ", " + photonVision.getPose3d());          
        }
        else
        {
          robotPose.set(Optional.empty());
        }

        break;

    case useLimelightVision:
        limelightVision.update();
        if (limelightVision.isFresh())
        {
          robotPose.set(Optional.of(new RobotPose(limelightVision.getTagID(), limelightVision.getTX(), limelightVision.getTY(), limelightVision.getPose3d())));
          // System.out.println((limelightVision.getSuggestResetOdometry() ?
          //   "reset odometry pose " : "addVisionMeasurement pose ") +
          //    limelightVision.getPose2d() + (limelightVision.isMegaTag2() ? " MegaTag2 pose" : " MegaTag pose"));
        }
        else
        {
          robotPose.set(Optional.empty());
        }

        break;

    default:
        System.out.println("impossible unless the enum is wrong");
        break;
    }
  }

  /**
   * Robot pose for whomever wants it
   * @return RobotPose
   */
  public Optional<RobotPose> getRobotPose()
  {
      return robotPose.get();
  }

  /**
   * 
   * @return location of the camera wrt the robot
   */
  public Transform3d getRobotToCamera()
  {
      return robotToCamera;
  }

  /**
   * Getter for vision system for CameraBase getters
   * @return
   */
  public PhotonVision getPhotonVision() {
    return photonVision;
  }

  /**
   * Getter for vision system for CameraBase getters
   * @return
   */
  public LimelightVision getLimelightVision() {
      return limelightVision;
  }

  /**
   * Getter for vision system for CameraBase getters
   * @return
   */
  public ControllerVision getControllerVision() {
      return controllerVision;
  }
}
