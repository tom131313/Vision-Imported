// https://luma.vision/ $250 hardware box runs free PhotonVision software
// limelightvision.io $450 integrated hardware and software
// photonvision.org free software and has some hardware recommendations
// WPILib vision
// https://danpeled.gitbook.io/synapse/ free vision software and has some hardware recommendations

// AcquireAprilTag significantly based on a WPILib example vision project so:
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Two example commands to align to the 2025 Reefscape reef targets at the #10 AprilTag:
 *   use 3-D pose
 *   use turn to point and drive a distance
 * Implements:
 *   Controller (roboRIO) 2-D & 3-D
 *   PhotonVision 2-D & 3-D
 *   LimeLight 2-D & 3-D
 */
package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    static
    {
        System.out.println("Loading: " + fullClassName);
    }

    public RobotContainer robotContainer;
    
    public Robot()
    {    
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() // called last in the periodic loop despite always seen written first in Robot.java
    {
        robotContainer.getVisionContainer().updateVision();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}
}
