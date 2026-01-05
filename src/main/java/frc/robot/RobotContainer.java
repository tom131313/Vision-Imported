package frc.robot;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.EnumSet;
import java.util.Enumeration;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToReefFieldRelativePose3D;
import frc.robot.commands.AlignToReefTagRelativeArcade2D;
import frc.robot.commands.AlignToReefTargetRelativeTransform3D;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CommandSchedulerLog;
import frc.robot.utils.CommandSchedulerLog.LogsSelector;
import frc.robot.vision.VisionContainer;
import frc.robot.vision.VisionContainer.VisionSelector;

/**
 * {@link Robot} gets you here and this starts the fan-out to everything and everywhere else
 */
public class RobotContainer {
    static {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    private CommandSchedulerLog schedulerLog;
    private static Drivetrain drivetrain = new Drivetrain();
    private static VisionContainer visionContainer;

    private static final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer() {
        // Caution that configDataLog and configCommandLogs may duplicate or conflict with each other.
        // They were written assuming that the other didn't exist which in general may or may not be true.
        // In this project they both exist so start the NT logging once as desired.
        configDataLog();

        configCommandLogs(EnumSet.of(LogsSelector.useConsole, LogsSelector.useDataLog)); // Select 1 or more different Command logging methods as desired
        // configCommandLogs(EnumSet.noneOf(LogsSelector.class)); // or none

        // use VisionSelector to choose which of the 3 vision systems to use
        var visionSelector = VisionSelector.usePhotonVision;
        DriverStation.reportWarning("vision selection " + visionSelector, false);
        visionContainer = new VisionContainer(visionSelector);

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
     * Logging to internal drive can cause insufficient space eventually.
     * Plug in a FAT32 USB drive for better space usage.
     */
    public static void configDataLog() {
        DataLogManager.start(); // default is log all NT

        // partial example (commented out) on how to select what to log

        // DataLogManager.logNetworkTables(false);

        // var dataLog = DataLogManager.getLog();

        // DriverStation.startDataLog(dataLog, true);

        // var networkTableInstance = NetworkTableInstance.getDefault();

        // networkTableInstance.startEntryDataLog(dataLog, "/FMSInfo", "NT:/FMSInfo");
        // networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.NETWORK_TABLE_NAME, "NT:/" + Constants.NETWORK_TABLE_NAME);
        // networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME, "NT:/" + Constants.ADVANTAGE_SCOPE_TABLE_NAME);
        // networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.Camera.CAMERA_1_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_1_BOT_POSE);
        // networkTableInstance.startEntryDataLog(dataLog, "/" + Constants.Camera.CAMERA_2_BOT_POSE, "NT:/" + Constants.Camera.CAMERA_2_BOT_POSE);
        // networkTableInstance.startEntryDataLog(dataLog, "/SmartDashboard", "NT:/SmartDashboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/Shuffleboard", "NT:/Shuffleboard");
        // networkTableInstance.startEntryDataLog(dataLog, "/LiveWindow", "NT:/LiveWindow");
        // networkTableInstance.startConnectionDataLog(dataLog, "NTConnection");
    }

    /**
     * Configure Command logging to Console/Terminal, DataLog, or ShuffleBoard
     */
    @SuppressWarnings("resource")
    public void configCommandLogs(EnumSet<LogsSelector> logsSelector) {
        if (!logsSelector.isEmpty()) {
            schedulerLog = new CommandSchedulerLog(logsSelector);
            schedulerLog.logCommandInitialize();
            schedulerLog.logCommandInterrupt();
            schedulerLog.logCommandFinish();
            schedulerLog.logCommandExecute(); // Can (optionally) generate a lot of output
        } else {
            new Alert("No logging", AlertType.kWarning).set(true);
        }
    }

    /**
     * useful if commands are organized in a different file than where VisionContainer is instantiated
     * 
     * @return an existing instance of VisionContainer
     */
    public static VisionContainer getVisionContainer() {
        return visionContainer;
    }

    /**
     * 
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

    /**
     * Enumerate all the network interfaces
     */
    public static String whatsMyIPAddress() {
        StringBuilder sb = new StringBuilder();
        try {
            // Get an enumeration of all network interfaces
            Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();

            // Iterate over each network interface
            while (interfaces.hasMoreElements()) {
                NetworkInterface networkInterface = interfaces.nextElement();

                // Skip loopback interfaces and interfaces that are down (optional but recommended)
                if (networkInterface.isLoopback() || !networkInterface.isUp()) {
                    continue;
                }

                sb.append("Interface: " + networkInterface.getDisplayName() + "\n");

                // Get an enumeration of all IP addresses bound to this interface
                Enumeration<InetAddress> addresses = networkInterface.getInetAddresses();

                // Iterate over each IP address
                while (addresses.hasMoreElements()) {
                    InetAddress address = addresses.nextElement();

                    // You can add further filtering here if needed (e.g., only IPv4 or IPv6)
                    sb.append("  IP Address: " + address.getHostAddress() + "\n");
                }
            }
        } catch (SocketException e) {
            sb.append("\nError retrieving network interface list: " + e.getMessage() + "\n");
        }

        return sb.toString();
    }
}
