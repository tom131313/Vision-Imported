package frc.robot;

import java.util.EnumSet;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Vision.Camera;
import frc.robot.utils.CommandSchedulerLog.CommandStageSelector;
import frc.robot.utils.CommandSchedulerLog.LogsSelector;
import frc.robot.vision.AcquireAprilTag;
import frc.robot.vision.VisionContainer.VisionSelector;

/**
 * This defines the user settable configuration
 * <p>
 * Add settings to the appropriate interface to limit visibility of those parameters
 * <p>
 * If a parameter is shared by more than one interface, then define a parameter in common in the
 * high level Config interface and parameters that reference that common parameter to all the
 * appropriate interfaces.
 * <p>
 * There are a few user settable constants scattered around this project especially in
 * {@link AcquireAprilTag}. These likely will not be changed and aren't in {@link Config} or
 * {@link Constants}.
 */
public interface Config {
    // variables used more than once can be put here and referenced in more than one interface
    // everything here is also visible by reference to the Config interface - they aren't hidden but
    // they don't have to be reference elsewhere.

    // public String commonlyUsed = value;

    /**
     * import static VisionSettings to see only these variables
     */
    public interface VisionSettings {

        public static VisionSelector visionSelector = VisionSelector.useControllerVision;//.usePhotonVision;

           /*
            * Select the camera model and resolution from the list of cameras that have calibration data.
            * 
            * Select if the Pose3D calculation is to be performed.
            * 
            * Enter the 3-D translation (location) of your camera wrt the bottom, center of
            * the robot.
            * 
            * Caution: roboRIO v1 has constrained memory and cpu.
            * 
            * For v1 use camera resolution about 320x240 maybe a little higher.
            * Experiment with your team usage.
            * 
            * "usePose3D" determines if the time consuming calculation for 3-D pose is
            * performed. If this setting is "false", then only the pitch and yaw of the robot
            * wrt the AprilTag is available and can they can be used for what LimeLight
            * describes as "servoing" mode. Note that the Pose3d is still "available" but it's
            * value is zero (origin; not useful). roboRIO v1 can do Pose3D but little time is
            * left for other processes. v1 memory limits AprilTag bitsCorrected to 1 error bit
            * (up to 2 on roboRIO v2)
            *
            * Select the camera device id for the ControllerVision.
            *
            * For real roboRIO, if only one camera, it doesn't matter which physical USB port
            * is used - both are 0.
            * 
            * If in Simulation mode on a Windows PC, usually this is correct for camera id:
            * If the external USB camera is plugged in at (before) boot-up time,
            * then it is cameraDeviceId = 0 after booting.
            * If the PC has an internal camera (common with laptops) and the
            * external camera is plugged in after boot-up, then the cameraDeviceId = 1.
            */

            // ControllerVision camera ID
            int cameraDeviceId = 0;

            // ControllerVision camera 
            Camera camera = Camera.ARDUCAM_320x240;

            // ControllerVision Pose3d calculation option
            boolean usePose3D = true;

            /*
            * for ControllerVision or PHotonVision enter the camera location wrt the robot. Example camera
            * mounted facing forward (zero radians yaw), half meter forward of center, quarter meter up
            * from center, zero meters left of center, pitch pointing up 25 degrees. (camera pointing up is
            * "-""; down is "+"")
            *
            * Note that the LimelightVision camera mount is set in the Limelight and is automatically
            * retrieved here
            * 
            * Caution: Limelight pitch setup has the opposite sign of PhotonVision and ControllerVision
            * LL positive pitch is pointing up
            * Snap Robot to Floor is only for MegaTag; MegaTag2 always snaps to floor.
            * Further than about 1.1m away from tag, MegaTag tends to have a lot of jitter especially in
            * the z (height) axis.
            */

            // ControllerVision camera mount
            Transform3d robotToCameraCV = new Transform3d(new Translation3d(0.5, 0.0, 0.25),
                new Rotation3d(0, Units.degreesToRadians(-25.), 0));

            // PhotonVision camera mount
            Transform3d robotToCameraPV = new Transform3d(new Translation3d(0.5, 0.0, 0.25),
                new Rotation3d(0, Units.degreesToRadians(-25.), 0));
            
            // PhotonVision camera name; select the camera name set in PV dashboard and camera mount
            String cameraNamePV = "Microsoft_LifeCam_HD-3000"; // "Arducam_OV9281_USB_Camera"

            // LimelightVision name that is set in the Limelight box through its dashboard
            String limelightName = "limelight";       
        }

    public interface CommandLoggingSettings {

        public static EnumSet<CommandStageSelector> commandStageSelector =
            EnumSet.allOf(CommandStageSelector.class);
        
        public static EnumSet<LogsSelector> logSelector =
            EnumSet.of(LogsSelector.useConsole, LogsSelector.useDataLog);
    }
}
