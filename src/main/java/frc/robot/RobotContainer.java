package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.VisionContainer.VisionSelector;

public class RobotContainer {
    static
    {
        System.out.println("Loading: " + java.lang.invoke.MethodHandles.lookup().lookupClass().getCanonicalName());
    }

    // Select as many different Command logging methods as desired
    private boolean useConsole            = true;
    private boolean useDataLog            = false;
    private boolean useShuffleBoardLog    = false;

    private static VisionContainer visionContainer;

    private static final CommandXboxController driverController = new CommandXboxController(0);

    public RobotContainer()
    {
        configDataLog();
        configCommandLogs();
        // use VisionSelector to choose which of the 3 vision systems to use
        var visionSelector = VisionSelector.usePhotonVision;
        DriverStation.reportWarning("vision selection " + visionSelector, false);
        visionContainer = new VisionContainer(visionSelector);
        configAButton();
        configBButton();
    }

    /**
     * activate 3d pose driving to target
     */
    private static void configAButton()
    {
        Trigger aButton = driverController.a();
        if (visionContainer.vision())
        {
            aButton.whileTrue(new AlignToReefFieldRelativePose3D(true, visionContainer));
        }
       else
        {
            aButton.whileTrue(Commands.print("No Vision").repeatedly());
        }
    }

    /**
     * activate 2-d pose driving to target (yaw and pitch)
     */
    private static void configBButton()
    {
        Trigger bButton = driverController.b();
        if (visionContainer.vision())
        {
            bButton.whileTrue(new AlignToReefTagRelativeArcade2D(visionContainer));
        }
        else
        {
            bButton.whileTrue(Commands.print("No Vision").repeatedly());
        }
    }

    /**
     * useful if commands are organized in a different file than where VisionContainer is instantiated
     * @return VisionContainer
     */
    public VisionContainer getVisionContainer()
    {
        return visionContainer;
    }

    /**
     * Select what to log with WPILib datalog
     */
    public static void configDataLog()
    {
        // logging to internal drive can cause insufficient space eventually.
        // plugin a FAT32 USB drive for better space usage.
        DataLogManager.start(); // default is log all NT

        // partial example (commented out) on how to select what to log
        // DataLogManager.logNetworkTables(false);
        // var dataLog = DataLogManager.getLog();
        // var networkTableInstance = NetworkTableInstance.getDefault();
        // DriverStation.startDataLog(dataLog, true);
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

    private CommandSchedulerLog schedulerLog;

    /**
     * Configure Command logging to Console/Terminal, DataLog, or ShuffleBoard
     */
    @SuppressWarnings("resource")
    public void configCommandLogs()
    {
        if (useConsole || useDataLog || useShuffleBoardLog) {
            schedulerLog = new CommandSchedulerLog(useConsole, useDataLog, useShuffleBoardLog);
            schedulerLog.logCommandInitialize();
            schedulerLog.logCommandInterrupt();
            schedulerLog.logCommandFinish();
            schedulerLog.logCommandExecute();  // Can (optionally) generate a lot of output        
        }
        else {
            new Alert("No logging", AlertType.kWarning).set(true);
        }
    }
}
