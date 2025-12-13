package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.concurrent.atomic.AtomicReference;

import org.opencv.core.Core;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * ControllerVision (roboRIO) returns pose of lowest tag ID
 *  (This probably needs work! See updateVision())
 * <p>PhotonVision returns best pose
 * <p>Limelight returns filtered selection from MegaTag or MegaTag2
 */
public class VisionContainer {
  private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
  static
  {
      System.out.println("Loading: " + fullClassName);
  }

  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME); // OpenCV    
  }

  PhotonVision photonVision;
  LimelightVision limelightVision;
  ControllerVision controllerVision;

  // Caution! Select only one vision usage! There is only one pose saved per iteration!
  // Each of the three Vision processes may be instantiated multiple times and
  // all three may run at the same time but not when used with this VisionContainer.

  private final boolean useControllerVision = true; // do 1 ControllerVision processing; select below camera and 3-D pose option and location
  private final boolean usePV = false; // do 1 PhotonVision processing; select below camera name and location
  private final boolean useLL = false; // do 1 LimeLight processing; select camera name

  AtomicReference<RobotPose> robotPose = new AtomicReference<>(new RobotPose(-1, 0., 0., Pose3d.kZero)); // this is the current pose
  private Transform3d robotToCamera;

  public VisionContainer()
  {

    try {
      Class.forName("frc.robot.AprilTagsLocations");
    } catch (ClassNotFoundException e) {
      e.printStackTrace();
    }


    /*****************************************************
     * Controller (roboRIO) camera used for AprilTag usage
     ****************************************************/
      // Select the camera model and resolution from the list.
      // Select if the Pose3D calculation is to be performed.
      // Enter the 3-D translation (location) of your camera wrt the bottom, center of the robot.
 
      // Caution: roboRIO v1 has constrained memory and cpu.

      // For v1 use camera resolution about 320x240 maybe a little higher. Experiment with your team usage.

      // "usePose3D" determines if the time consuming calculation for 3-D pose is performed.
      // If this setting is "false", then only the pitch and yaw of the robot wrt the AprilTag is
      // available and can they can be used for what LimeLight describes as "servoing" mode.
      // roboRIO v1 can do Pose3D but little time is left for other processes.
      // v1 memory limits AprilTag bitsCorrected to 1 error bit (up to 2 on roboRIO v2)

      // Example camera mounted facing forward, half a meter forward of center, quarter a meter up from center
      // pointing up 25 degrees.

      boolean usePose3D = true;
      var robotToCameraCV = new Transform3d(new Translation3d(0.5, 0.0, 0.25), new Rotation3d(0,Units.degreesToRadians(-25.),0));
      robotToCamera = robotToCameraCV;

      controllerVision = useControllerVision ?
          new ControllerVision(0, ControllerVision.CameraOption.ArduCam320x240 /*LifeCam640x480*/, usePose3D, robotToCameraCV) :
          null;
      if (controllerVision != null)
      {
          Thread acquireControllerCamera = new Thread(controllerVision);
          acquireControllerCamera.setName("roboRIOCameraPose");
          acquireControllerCamera.setPriority(acquireControllerCamera.getPriority() - 1);
          acquireControllerCamera.setDaemon(true);
          acquireControllerCamera.start();        
      }


    /****************************************************
     * PhotonVision camera used for AprilTag usage
     ***************************************************/
    // change this to match the camera name and location of your camera
    //Forward Camera
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    // var robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));
    // Microsoft_LifeCam_HD-3000   Arducam_OV9281_USB_Camera_(A)  Arducam_OV9281_USB_Camera_(B)
  
    var cameraName = "Microsoft_LifeCam_HD-3000";
    var robotToCameraPV = new Transform3d(new Translation3d(0.5, 0.0, 0.25), new Rotation3d(0,Units.degreesToRadians(-25.),0));
    robotToCamera = robotToCameraPV;

    photonVision = usePV ? new PhotonVision(cameraName, robotToCameraPV) : null;


    /****************************************************
     * LimeLight camera used for AprilTag usage
     ***************************************************/
    /*
     * Caution: Limelight pitch setup has the opposite sign of PhotonVision and ControllerVision
     * LL positive pitch is pointing up
     * Snap Robot to Floor is only for MegaTag; MegaTag2 always snaps to floor.
     * Further than 1.1m MegaTag tends to have a lot of jitter especially in the z (height) axis.
     */
    var limelightName = "limelight";

    limelightVision = useLL ? LimelightVision.makeCamera(limelightName) : null;

    if (limelightVision != null)
    {
        var c = LimelightHelpers.getCameraPose3d_RobotSpace(limelightName); // retrieve camera position in LL
        var robotToCameraLL = new Transform3d(
              c.getX(), c.getY(), c.getZ(),
              new Rotation3d(c.getRotation().getX(), c.getRotation().getY(), c.getRotation().getZ())); 
        robotToCamera = robotToCameraLL;

        // LimelightHelpers.setStreamMode_PiPSecondary(limelightName); // if there is a driver camera, where to put the image - pick your favorite
    }
  }

  public void updateVision()
  {
    if (controllerVision != null)
    {
        controllerVision.update();
        if (controllerVision.isFresh())
        {
          if (controllerVision.getPoses().size() >= 1)
          {
            robotPose.set(controllerVision.getPoses().get(0).clone()); // arbitrarily pick lowest tag id
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
        // System.out.println(LLTeam);
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
   * @return 
   */
  public RobotPose getRobotPose()
  {
    return robotPose.get().clone(); // fresh copy pulled from the AtomicReference
  }

  public Transform3d getRobotToCamera()
  {
      return robotToCamera;
  }

  public PhotonVision getPhotonVision() {
      return photonVision;
  }

  public LimelightVision getLimelightVision() {
      return limelightVision;
  }

  public ControllerVision getControllerVision() {
      return controllerVision;
  }
}
