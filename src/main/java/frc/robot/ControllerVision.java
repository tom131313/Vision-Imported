// Detecting and drawing AprilTags is significantly based on a WPILib example vision project so
// for ControllerVision.java, AcquireAprilTag.java, and AcquireRobotPose.java much is:
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;

import org.opencv.core.MatOfDouble;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
 * camera, then any detected AprilTags are marked up on the image, transformed to the robot on
 * the field pose and sent to NetworkTables. The robot pose is clamped to the floor and that could
 * be removed in AcquireRobotPose. (LimelightVision setup has that option as set in the Limelight.)
 * AcquireRobotPose has no provision for using the gyro to improve pose estimation. The gyro
 * heading could be used instead of the pose rotation, if desired.
 *
 * <p>Be aware that the performance on this is much worse than a coprocessor solution!
 * 
 * <p>The camera view is displayed with additional information of the AprilTag pose to camera.
 * That display is optional and a tiny bit of cpu processing can be saved by not doing it.
 * Since AprilTag view can be in normal light, that camera can also be used by the operator if it's
 * pointing in a good direction. If the operator doesn't need that view, don't display it and
 * the image can be made a little darker and more contrast - whatever can reduce the cpu
 * processing time for an image. Experiment with exposure, contrast, gamma, brightness, etc.
 * 
 * <p>AprilTag has known pose on the field loaded from file (WPILib or your custom file).
 * Detected tag's perspective seen by the camera is used to calculate an estimate of the camera pose
 * relative to the tag.
 * Camera has a known pose relative to the robot chassis.
 * Combine this chain to calculate the robot pose in the field.
 * Camera parameters must be provided from another source source as the related calibration program.
*/     
public class ControllerVision extends CameraBase implements Runnable {
  private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
  static
  {
      System.out.println("Loading: " + fullClassName);
  }

    public enum CameraOption{ArduCam320x240, ArduCam1280x800, LifeCam320x240, LifeCam640x480};

    int cameraDeviceId;

    // Calibrate the camera at the used resolution or scale Fx,Fy,Cx,Cy proportional
    // to what resolution was used for camera calibration.
    int cameraW; // image width
    int cameraH; // image height
    int fps;     // frames/second
    double cameraFx; // fx camera horizontal focal length, in pixels
    double cameraFy; // fy camera vertical focal length, in pixels
    double cameraCx; // cx camera horizontal focal center, in pixels
    double cameraCy; // cy camera vertical focal center, in pixels
    MatOfDouble distortionCoeffs;
    public Image image = new Image(); // where a video frame goes for multiple processes to use
    AcquireAprilTag acquireAprilTag;
    AcquireRobotPose acquireRobotPose;
    ArrayList<RobotPose> poses;

    public ControllerVision(int cameraDeviceId, CameraOption selectedCameraOption, boolean usePose3D, Transform3d robotToCamera)
    {
        this.cameraDeviceId = cameraDeviceId;

      switch(selectedCameraOption) {
        case ArduCam320x240: // rough calibration - wasn't done with a nice flat board
        cameraW = 320;
        cameraH = 240;
        fps = 100;
        cameraFx = 273.8682279422785;
        cameraFy = 274.2578211409246;
        cameraCx = 142.187975375679;
        cameraCy = 124.6151823259089;
        distortionCoeffs = new MatOfDouble();
          // much of the small amount of distortion from calibration was actually the board
          // not being smooth so don't bother using the distortion
          // [0.03872533667096114, -0.2121025605447465, 0.00334472765894009, -0.006080540135581289, 0.4001779842036727]
          break;

  // {"DISPLAY_NAME":"","DISTORTION_COEFFICIENTS":[
  // 0.09279791624496432,-0.013162736031556143,0.00034288948646321224,0.0008282135215583248,-0.2112763216404896],
  // "INTRINSICS_MATRIX":[
  // 742.0300917168702,0.0,652.0266479431351,
  // 0.0,741.6376375442737,375.18558295593374,
  // 0.0,0.0,1.0],
  // "REPROJECTION_ERROR":0.31303468947665714,"RES_X":1280.0,"RES_Y":800.0}
        case ArduCam1280x800:// Arducam_OV9281_USB_Camera_(A?); Cameron board 1280x800      
        cameraW = 1280;
        cameraH = 800;
        fps = 100;
        cameraFx = 907.6920444758049;
        cameraFy = 907.1513951038395;
        cameraCx = 604.1750223777503;
        cameraCy = 416.4609913313957;
        distortionCoeffs = new MatOfDouble(
            0.040354289830866516, -0.044066115475547216, 6.662818829158613E-4,9.755603732755772E-4,  // k1 k2 p1 p2
              -0.013630390510289322, // k3
              -0.0011985508423857224, 0.003370423168524356, 0.0010337869630847195); // k4 k5 k6
              // assume s1 s2 s3 s4 tx ty are all zeros
          break;

        case LifeCam320x240:
          // 320x240 lifecam calibration from PhotonVision
          cameraW = 320;
          cameraH = 240;
          fps = 30;
          cameraFx = 353.74653217742724;
          cameraFy = 340.77624878700817;
          cameraCx = 163.5540798921191;
          cameraCy = 119.8945718300403;
          distortionCoeffs = new MatOfDouble();
          break;

        case LifeCam640x480:
          // 640x480 lifecam calibration from WPILib example
          cameraW = 640;
          cameraH = 480;
          fps = 30;
          cameraFx = 699.3778103158814;
          cameraFy = 677.7161226393544;
          cameraCx = 345.6059345433618;
          cameraCy = 207.12741326228522;
          distortionCoeffs = new MatOfDouble();
          break;

        default: break;
      }

      acquireAprilTag = new AcquireAprilTag(this);
      acquireRobotPose = new AcquireRobotPose(this, usePose3D, robotToCamera);
    }

    /**
     * Loop of the thread to acquire camera images and compute the poses of AprilTags
     */
    public void run()
    {
      // This 'while' cannot be 'true'. The program will never exit if it is. Using
      // interrupted() lets the robot stop this thread when restarting robot code or deploying.
      while (!Thread.interrupted()) {
        acquireAprilTag.run();
        acquireRobotPose.run();
      }
    }

    /**
     * Get latest data - must be first before accessing data
     */
    @Override
    public void update() {
        poses = acquireRobotPose.getPoses();
    }

    /**
     * check for new data 
     * @return true if new data; false if stale or no data
     */
    @Override
    public boolean isFresh() {
        return poses != null;
    }

    // These are single-valued characteristics in a possibly multi-tag frame.
    // Arbitrarily pick the first (lowest number from detection?) tag id in the list.
    // A different sort or individually selected data by tag ID could be added.
    // Or use the getPoses() method and parse from it anything you want.

    /**
     * @return 3-D pose based on lowest tag id used
     */
    @Override
    public Pose3d getPose3d() {
      return poses.get(0).pose3D;
    }

    /**
     * @return 2-D pose based on the lowest tag id used
     */
    @Override
    public Pose2d getPose2d() {
        return new Pose2d(poses.get(0).pose3D.getX(), poses.get(0).pose3D.getY(), new Rotation2d(poses.get(0).pose3D.getRotation().getZ()));
    }

    /**
     * @return robot yaw to tag based on the lowest tag id used
     */
    @Override
    public double getTX() {
      return poses.get(0).yaw;
    }

    /**
     * @return robot pitch to tag based on the lowest tag id used
     */
    @Override
    public double getTY() {
      return poses.get(0).pitch;
    }

    /**
     * @returns lowest tag id used
     */
    @Override
    public int getTagID() {
      return poses.get(0).AprilTagId;
    }

    /**
     * 
     * @return list of poses from all tag ids used
     */
    public ArrayList<RobotPose> getPoses() {
      return poses;
    }

}
      // Set up Pose Estimator - parameters are for a Microsoft LifeCam HD-3000
      // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
      
      // theoretically the resolution factor also directly effects the other camera parameters
      // but apparently recalibrating at various resolutions does yield slightly varying results.

      // the other arducam with Cameron board
      // 902.229862551387, 0.0, 630.0164752216564,
      // 0.0, 901.4463862207839, 413.07035838353374,
      // 0.0, 0.0, 1.0
          
      // using Samsung tablet
      // camMatrix: 
      // [908.014390012331, 0.0, 597.8814589360088,
      // 0.0, 908.6191476725982, 413.8612971221383,
      //  0.0, 0.0, 1.0]
      // distortionCoeffs:
      // [0.052173675342917676, -0.07090132559762727, -5.866285222375848E-4, -0.002475170189898108, 0.012835458369393258, -0.002442550780537089, 0.005625375255529692, 9.391199615674332E-4]

      // using samsung desktop screen
      // CALIBRATION SUCCESS for res 1280x800 in 291.6018ms! camMatrix: 
      // [907.5869786612509, 0.0, 604.8488080504388, 0.0, 909.4739674568915, 411.6013865871937, 0.0, 0.0, 1.0]
      // distortionCoeffs:
      //  0.05483635427166114, k1
      // -0.09776533602636311, k2
      // -9.78236645068978E-5, p1
      //  4.8823267235786116E-5, p2
      //  0.04332976089574157, k3
      // -0.00373311108258594, k4
      //  0.005395356105023349, k5
      // -1.99252152117131E-4 k6
      // (1 + 0.05483635427166114r^2 + -0.09776533602636311r^4 + 0.04332976089574157r^6) / (1 + -0.00373311108258594r^2 + 0.005395356105023349r^4 + -1.99252152117131E-4r^6) for r from 0 to 400

      //opencv tutorial example
      // https://www.wolframalpha.com/input/?i=plot (1 + -4.1802327176423804e-001 Power[\(40)Divide[r,6.5746697944293521e+002]\(41),2] + 5.0715244063187526e-001 Power[\(40)Divide[r,6.5746697944293521e+002]\(41),4] + -5.7843597214487474e-001*Power[\(40)Divide[r,6.5746697944293521e+002]\(41),6]) 
      // all times r
      // for r from 0 to 400
