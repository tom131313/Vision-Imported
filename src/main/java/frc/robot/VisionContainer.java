package frc.robot;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ControllerVision (roboRIO) returns team specified pose of lowest tag ID
 *  (This probably needs work! See {@link #periodic()} ControllerVision block)
 * <p>PhotonVision returns its best pose
 * <p>LimelightVision returns team specified filtered selection from MegaTag or MegaTag2
 *  (This is a reasonable default but could use work! See {@link LimelightVision#update()})
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

  private ControllerVision controllerVision = null; // only for Controller (roboRIO)
  private PhotonVision photonVision = null; // only for PV
  private LimelightVision limelightVision = null; // only for LL

  private boolean vision = false;
  public enum VisionSelector{useControllerVision, usePhotonVision, useLimelightVision}
  public VisionSelector visionSelector;

  private AtomicReference<RobotPose> robotPose = new AtomicReference<>(new RobotPose(-1, 0., 0., Pose3d.kZero)); // this is the current pose
  private Transform3d robotToCamera;

  public VisionContainer(VisionSelector visionSelector)
  {
    this.visionSelector = visionSelector;

    if (visionSelector == VisionSelector.useControllerVision)
    {
    /*****************************************************
     * Controller (roboRIO) camera used for AprilTag usage
     ****************************************************/
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
      }
      else
      {
        controllerVision = null;
      }
    }

    if (visionSelector == VisionSelector.usePhotonVision)
    {
      /****************************************************
       * PhotonVision camera used for AprilTag usage
       ***************************************************/
      // Select the camera name

      // Enter the camera location wrt the robot. Example camera mounted facing
      // forward, half a meter forward of center, quarter a meter up from center
      // pointing up 25 degrees.

      var cameraName = "Microsoft_LifeCam_HD-3000";
      var robotToCameraPV = new Transform3d(new Translation3d(0.5, 0.0, 0.25),
                new Rotation3d(0,Units.degreesToRadians(-25.),0));
      robotToCamera = robotToCameraPV;

      photonVision = new PhotonVision(cameraName, robotToCameraPV);
      if (! photonVision.camera.isConnected())
      {
        photonVision.camera.getAllUnreadResults(); // force PV to issue no camera by that name but there are other PV cameras
        photonVision = null; // put it back to null since constructor didn't really make a camera
      }
    }

    if (visionSelector == VisionSelector.useLimelightVision)
    {
      /****************************************************
       * LimeLight camera used for AprilTag usage
       ***************************************************/
      /*
      * Caution: Limelight pitch setup has the opposite sign of PhotonVision and ControllerVision
      * LL positive pitch is pointing up
      * Snap Robot to Floor is only for MegaTag; MegaTag2 always snaps to floor.
      * Further than 1.1m MegaTag tends to have a lot of jitter especially in the z (height) axis.
      */
      // enter the limelight name
      var limelightName = "limelight";
      if (LimelightVision.isAvailable(limelightName))
      {
          limelightVision = new LimelightVision(limelightName);
          var c = LimelightHelpers.getCameraPose3d_RobotSpace(limelightName); // retrieve camera position in LL
          var robotToCameraLL = new Transform3d(
                c.getX(), c.getY(), c.getZ(),
                new Rotation3d(c.getRotation().getX(), c.getRotation().getY(), c.getRotation().getZ())); 
          robotToCamera = robotToCameraLL;
          // put other LL setup as needed
          // LimelightHelpers.setStreamMode_PiPSecondary(limelightName); // if there is a driver camera, where to put the image - pick your favorite
      }
      else
      {
          limelightVision = null;
      }
    }

    vision = controllerVision != null || photonVision != null || limelightVision != null;
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
    if (controllerVision != null)
    {
        controllerVision.update();
        if (controllerVision.isFresh())
        {
          if (controllerVision.getPoses().size() >= 1)
          {
            //FIXME arbitrarily pick lowest tag id; this needs to be thought through
            robotPose.set(controllerVision.getPoses().get(0).clone());
            // controllerVision.getPoses().forEach(pose -> System.out.println("CV " + pose)); // show all available data
          }
        }
        else
        {
          robotPose.set(new RobotPose(-1, 0., 0., new Pose3d()));
        }
    }

    if (photonVision != null)
    {
        photonVision.update();
        if (photonVision.isFresh())
        {
          robotPose.set(new RobotPose(photonVision.getTagID(), photonVision.getTX(), photonVision.getTY(), photonVision.getPose3d()));
          // System.out.println("PV " + photonVision.getTagID() + ", " + photonVision.getTX() + ", " + photonVision.getTY() + ", " + photonVision.getPose3d());          
        }
        else
        {
          robotPose.set(new RobotPose(-1, 0., 0., new Pose3d()));
        }
    }

    if (limelightVision != null)
    {
        limelightVision.update();
        if (limelightVision.isFresh())
        {
          robotPose.set(new RobotPose(limelightVision.getTagID(), limelightVision.getTX(), limelightVision.getTY(), limelightVision.getPose3d()));
          // System.out.println((limelightVision.getSuggestResetOdometry() ?
          //   "reset odometry pose " : "addVisionMeasurement pose ") +
          //    limelightVision.getPose2d() + (limelightVision.isMegaTag2() ? " MegaTag2 pose" : " MegaTag pose"));
        }
        else
        {
          robotPose.set(new RobotPose(-1, 0., 0., new Pose3d()));
        }
    }
  }

  /**
   * Robot pose for whomever wants it
   * @return RobotPose
   */
  public RobotPose getRobotPose()
  {
    return robotPose.get().clone(); // fresh copy pulled from the AtomicReference
  }

  /**
   * 
   * @return location of the camera wrt the robot
   */
  public Transform3d getRobotToCamera()
  {
      return robotToCamera;
  }
}
