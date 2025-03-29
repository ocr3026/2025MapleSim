// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.autonomous.AutoBase.Paths;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

public class Robot extends LoggedRobot {
	private Command autonomousCommand;
	private final RobotContainer robotContainer;

	DriverStation.Alliance lastAlliance = Alliance.Blue;
	int timesRecompiled = 0;

	public Robot() {
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}

		switch (Constants.currentMode) {
			case REAL:
				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case SIM:
				Logger.addDataReceiver(new NT4Publisher());
				break;

			case REPLAY:
				setUseTiming(false);
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}

		Logger.registerURCL(URCL.startExternal());

		Logger.start();

		robotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		Threads.setCurrentThreadPriority(true, 99);

		CommandScheduler.getInstance().run();

		Threads.setCurrentThreadPriority(false, 10);
	}

	@Override
	public void disabledInit() {
		robotContainer.resetSimulationField();
	}

	@Override
	public void disabledPeriodic() {

		autonomousCommand = robotContainer.getAutonomousCommand();

		if (DriverStation.getAlliance().orElse(Alliance.Blue) != lastAlliance) {
			timesRecompiled++;
			Paths.flipCoralPoses();
			robotContainer.recompileAutos();
			SmartDashboard.putString("hasRecompiled", "Recompiled: " + timesRecompiled + " times");
			Paths.updateStoredChooser();
			lastAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
		} else if (Paths.recompileNeeded()) {
			timesRecompiled++;
			robotContainer.recompileAutos();
			SmartDashboard.putString("hasRecompiled", "Recompiled: " + timesRecompiled + " times");
			Paths.updateStoredChooser();
		}
	}

	@Override
	public void autonomousInit() {
		// autonomousCommand = robotContainer.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/** @param teleopInit( */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {
		robotContainer.updateSimulation();
	}
}
