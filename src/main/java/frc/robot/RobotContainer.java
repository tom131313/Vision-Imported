package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Config.CommandLoggingSettings;
import frc.robot.commands.AlignToReefFieldRelativePose3D;
import frc.robot.commands.AlignToReefTagRelativeArcade2D;
import frc.robot.commands.AlignToReefTargetRelativeTransform3D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandSchedulerLog;
import frc.robot.vision.VisionContainer;

/**
 * {@link Robot} gets you here and this starts the fan-out to everything and everywhere else
 */
public class RobotContainer  {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private static Drivetrain drivetrain = new Drivetrain();
    private static VisionContainer visionContainer;

    private static final CommandXboxController driverController =
        new CommandXboxController(Constants.driverController);

    public RobotContainer() {

        configDataLog();

        new CommandSchedulerLog(CommandLoggingSettings.commandStageSelector,
                CommandLoggingSettings.logSelector);

        visionContainer = new VisionContainer();

        configAButton();
        configBButton();
        configXButton();
    }

    /**
     * activate 3d pose driving to target using field coordinates
     */
    private static void configAButton() {
        Trigger aButton = driverController.a();
        if (getVisionContainer().vision()) {
            aButton.whileTrue(new AlignToReefFieldRelativePose3D(true, getVisionContainer(), getDrivetrain()));
        } else {
            aButton.whileTrue(Commands.print("No Vision").repeatedly());
        }
    }

    /**
     * activate 2-d pose driving to target (yaw and pitch)
     */
    private static void configBButton() {
        Trigger bButton = driverController.b();
        if (getVisionContainer().vision()) {
            bButton.whileTrue(new AlignToReefTagRelativeArcade2D(getVisionContainer(), getDrivetrain()));
        } else {
            bButton.whileTrue(Commands.print("No Vision").repeatedly());
        }
    }

    /**
     * activate 3d pose driving to target using camera coordinates
     */
    private static void configXButton() {
        Trigger xButton = driverController.x();
        if (getVisionContainer().vision()) {
            xButton.whileTrue(new AlignToReefTargetRelativeTransform3D(true, visionContainer, getDrivetrain()));
        } else {
            xButton.whileTrue(Commands.print("No Vision").repeatedly());
        }
    }

    /**
     * Select what to log with WPILib datalog
     * <p>
     * Caution that {@link CommandSchedulerLog} assumes its logging is turned on and not suppressed
     * by interactions with {@link #configDataLog}
     * <p>
     * Logging to internal drive can cause insufficient space eventually.
     * Plug in a FAT32 USB drive for better space usage.
     */
    public static void configDataLog() {
        // FIXME select data log options
        DataLogManager.start(); // default is log all NT

        // // partial example (commented out) on how to select what to log
        // // logging of the "Constants." are illustrative of what could be done and are not in this project
        // DataLogManager.logNetworkTables(false);
        // var dataLog = DataLogManager.getLog();
        // DriverStation.startDataLog(dataLog, true);
        // var networkTableInstance = NetworkTableInstance.getDefault();
        // networkTableInstance.startEntryDataLog(dataLog, "/FMSInfo", "NT:/FMSInfo");
        // networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.NETWORK_TABLE_NAME, "NT:/" + Constants.NETWORK_TABLE_NAME);
        // networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME, "NT:/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        // networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.Camera.CAMERA_1_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_1_BOT_POSE);
        // networkTableInstance.startEntryDataLog(dataLog, "/SmartDashboard", "NT:/SmartDashboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/Shuffleboard", "NT:/Shuffleboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/LiveWindow", "NT:/LiveWindow");
        // networkTableInstance.startConnectionDataLog(dataLog, "NTConnection");
    }

    /**
     * Class getters are useful if commands that reference them are organized in different files
     * than where the classes are instantiated
     */

    /**
     * @return an existing instance of VisionContainer
     */
    public static VisionContainer getVisionContainer() {
        return visionContainer;
    }

    /**
     * @return an existing instance of Drivetrain
     */
    public static Drivetrain getDrivetrain() {
        return drivetrain;
    }

    /**
     * {@link Robot#robotPeriodic()} will call this method before the command scheduler and this
     * method runs all the rest of the methods that must be run before the command scheduler
     */
    public static void runBeforeCommands() {
        getVisionContainer().update();
    }
}
