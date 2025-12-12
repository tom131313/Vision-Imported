package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
/**
 * Type class for passing the robot pose around between methods.
 * Deep copy made of the constructor arguments.
 */
public class RobotPose {
    int AprilTagId;
    double yaw; // degrees
    double pitch; // degrees
    Pose3d pose3D; // meters and radians

    public RobotPose(int AprilTagId, double yaw, double pitch, Pose3d pose3D) {
        // deep copy of the arguments
        this.AprilTagId = AprilTagId;
        this.yaw = yaw;
        this.pitch = pitch;
        this.pose3D = new Pose3d(
            pose3D.getX(), pose3D.getY(), pose3D.getZ(),
                new Rotation3d(pose3D.getRotation().getX(), pose3D.getRotation().getY(), pose3D.getRotation().getZ()));
    }

    /**
     * deep copy of object based on constructor
     */
    public RobotPose clone()
    {
        return new RobotPose(AprilTagId, yaw, pitch, pose3D);
    }

    public String toString() {
        return String.format("tag %d, yaw [deg] %f, pitch [deg] %f, %s", AprilTagId, yaw, pitch, pose3D);
    }
}

