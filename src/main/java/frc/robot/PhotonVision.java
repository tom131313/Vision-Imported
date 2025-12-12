package frc.robot;

import java.lang.invoke.MethodHandles;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

public class PhotonVision  extends CameraBase {
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    // Tag positions
    // tag rotation is CCW looking down on field from the ceiling.
    // rotating around Z, 0 degrees is parallel to Y and facing down field or +X. 30 degrees is still
    // facing down field +X and a little facing into the +Y across the field

    final boolean CustomTagLayout = false; // true is use custom deploy of layout

    PhotonCamera camera;
    boolean targetVisible;
    PhotonTrackedTarget bestTarget;
    PhotonPoseEstimator photonPoseEstimator;
    Transform3d cameraInRobotFrame;

    int maxTagId = 25    +1; // add 1 for tag ID 0
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("PhotonVisionCamera");
    
    List<StructPublisher<Pose3d>> publishRobotPose = new ArrayList<>(maxTagId); // Pose3d
    List<DoubleArrayPublisher> publishRobotServoing = new ArrayList<>(maxTagId); // yaw and pitch
    ArrayList<RobotPose> poses = new ArrayList<>(maxTagId);
    
    // PhotonPipelineResult PVResult = new PhotonPipelineResult();

    public PhotonVision(String name, Transform3d cameraInRobotFrame/*robotToCamera transform 3d*/) {
      this.cameraInRobotFrame = cameraInRobotFrame;
      camera = new PhotonCamera(name);

      // make an empty bucket for every possible tag
      for (int tag = 0; tag < maxTagId; tag++) {
        publishRobotPose.add(null);
        var robotPosePublisher = tagsTable.getStructTopic("robotPose3D_" + tag, Pose3d.struct).publish();
        publishRobotPose.set(tag, robotPosePublisher);

        publishRobotServoing.add(null);
        var robotServoingPublisher = tagsTable.getDoubleArrayTopic("robotServoing_ + tag").publish();
        publishRobotServoing.set(tag, robotServoingPublisher);
      }

      // better poses possible using the PhotonPoseEstimator; lot's of possibilities with it
      // PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cameraInRobotFrame);
    }

    /**
     * Read in relevant data from the Camera
     */
    public void update() {

    targetVisible = false;
    var results = camera.getAllUnreadResults();

    if (!results.isEmpty()) {
        var result = results.get(results.size() - 1); // Get the last, most current result in the list.
        if (result.hasTargets()) { // At least one AprilTag was seen by the camera

            // example looping through all targets seen in this frame. An individual tag information
            // could be pulled off here as the PhotonVision example program shows.
            // var countTargets = 0;
            // for (var target : result.getTargets()) {
            //   System.out.println("target " + ++countTargets + " " + target);
            // }

            // for this example use the PV designated Best Target
            bestTarget = result.getBestTarget();
            // double area = bestTarget.getArea();
            // double skew = bestTarget.getSkew();
            // List<TargetCorner> corners = bestTarget.getDetectedCorners();
            // double poseAmbiguity = bestTarget.getPoseAmbiguity();
            // Transform3d alternateCameraToTarget = bestTarget.getAlternateCameraToTarget();

            // https://docs.photonvision.org/en/v2025.0.0-beta-4/docs/apriltag-pipelines/multitag.html
             //Enabling MultiTag

            // if (result.getMultiTagResult().isPresent()) {
            //   Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
            // }

            // Correct pose estimate with vision measurements
        // var visionEst = vision.getEstimatedGlobalPose();
        // visionEst.ifPresent(
        //         est -> {
        //             // Change our trust in the measurement based on the tags we can see
        //             var estStdDevs = vision.getEstimationStdDevs();

        //             drivetrain.addVisionMeasurement(
        //                     est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
        //         });

            // System.out.println(/*"\nRESULT " + result +*/ "\n\nPV BEST TARGET " + bestTarget);
            var tag = bestTarget.getFiducialId();
            publishRobotPose.get(tag).set(getPose3d());
            publishRobotServoing.get(tag).set(new double[]{bestTarget.getYaw(), bestTarget.getPitch()});

            targetVisible = true;

        }
        else {
                  System.out.println("skipping because result has no targets");
        }
      }
    }

    @Override
    public boolean isFresh() {
      return targetVisible;
    }

    @Override
    public Pose3d getPose3d() {
      Pose3d tagInFieldFrame = AprilTagsLocations.getTagLocation(getTagID());
      var tagInCameraFrame = bestTarget.getBestCameraToTarget();
      var robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame,  tagInCameraFrame,  cameraInRobotFrame);
      return robotInFieldFrame;
    }

    @Override
    public Pose2d getPose2d() {
        return new Pose2d(getPose3d().getX(), getPose3d().getY(), new Rotation2d(getPose3d().getRotation().getZ()));
    }

    @Override
    public double getTX() {
      return bestTarget.getYaw();
    }

    @Override
    public double getTY() {
      return bestTarget.getPitch();
    }

    @Override
    public int getTagID() {
      return bestTarget.getFiducialId();
    }

//FIXME
// call this then use it in addVisionMeasurement
// https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update(); //FIXME need "update(result)" - which one if multiple; also need previous pose passed in
    // }
}
