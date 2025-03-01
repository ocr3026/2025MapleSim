// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.wrist.WristConstants.intakeVoltage;
import static frc.robot.subsystems.wrist.WristConstants.outtakeVoltage;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.autonomous.TestAuto;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.climber.ClimberSparkIO;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSpark;
import frc.robot.subsystems.wrist.WristSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
	private final ElevatorSubsystem elevatorSubsystem;
	private final WristSubsystem wristSubsystem;
	private final ClimberSubsystem climberSubsystem;
	private final Drive drive;
	private final Vision vision;
	private SwerveDriveSimulation driveSimulation = null;

	private final CommandJoystick translationJoystick = new CommandJoystick(0);
	private final CommandJoystick rotationJoystick = new CommandJoystick(1);
	public static final CommandXboxController xbox = new CommandXboxController(2);

	private final LoggedDashboardChooser<Command> autoChooser;

	public RobotContainer() {
		SmartDashboard.putNumber("delayStartTime", 0);
		switch (Constants.currentMode) {
			case REAL:
				SmartDashboard.putString("currentRobotMode", "REAL");
				SmartDashboard.putString("currentRobotMode", "REAL");
				drive = new Drive(
						new GyroIONavX(),
						new ModuleIOSpark(0),
						new ModuleIOSpark(1),
						new ModuleIOSpark(2),
						new ModuleIOSpark(3),
						(pose) -> {});

				elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSpark());
				wristSubsystem = new WristSubsystem(new WristIOSpark());

				vision = new Vision(
						drive, new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));

				climberSubsystem = new ClimberSubsystem(new ClimberSparkIO());
				break;
			case SIM:
				SmartDashboard.putString("currentRobotMode", "SIM");

				SmartDashboard.putString("currentRobotMode", "SIM");

				this.driveSimulation =
						new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(0, 0, new Rotation2d()));

				elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
				wristSubsystem = new WristSubsystem(new WristIOSim());

				SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

				drive = new Drive(
						new GyroIOSim(driveSimulation.getGyroSimulation()),
						new ModuleIOSim(driveSimulation.getModules()[0]),
						new ModuleIOSim(driveSimulation.getModules()[1]),
						new ModuleIOSim(driveSimulation.getModules()[2]),
						new ModuleIOSim(driveSimulation.getModules()[3]),
						driveSimulation::setSimulationWorldPose);

				vision = new Vision(
						drive,
						new VisionIOPhotonVisionSim(
								VisionConstants.camera0Name,
								VisionConstants.robotToCamera0,
								driveSimulation::getSimulatedDriveTrainPose),
						new VisionIOPhotonVisionSim(
								VisionConstants.camera1Name,
								VisionConstants.robotToCamera1,
								driveSimulation::getSimulatedDriveTrainPose));

				climberSubsystem = new ClimberSubsystem(new ClimberSparkIO());
				break;
			default:
				SmartDashboard.putString("currentRobotMode", "DEFAULT");

				drive = new Drive(
						new GyroIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						new ModuleIO() {},
						(pose) -> {});

				elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {});
				wristSubsystem = new WristSubsystem(new WristIO() {});
				vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

				climberSubsystem = new ClimberSubsystem(new ClimberSparkIO());
				break;
		}

		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

		autoChooser.addOption("TestAuto", TestAuto.returnTest());

		autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

		configureBindings();
	}

	private void configureBindings() {
		drive.setDefaultCommand(DriveCommands.joystickDrive(
				drive, translationJoystick::getY, translationJoystick::getX, () -> -rotationJoystick.getX()));

		translationJoystick.button(11).onTrue(Commands.runOnce(drive::stopWithX, drive));

		final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
				? () -> drive.resetOdometry(
						driveSimulation
								.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
				: () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

		translationJoystick.button(12).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

		xbox.b().whileTrue(ClimberCommands.moveClimber(climberSubsystem, -.5));
		xbox.a().whileTrue(ClimberCommands.moveClimber(climberSubsystem, .5));
		//climberSubsystem.setDefaultCommand(ClimberCommands.moveClimber(climberSubsystem, xbox.getLeftY()));
		//	xbox.y().whileTrue(ClimberCommands.autoPositionClimber(climberSubsystem, 45)
		//	.andThen(ClimberCommands.autoPositionClimber(climberSubsystem, 135)));
		xbox.a().whileTrue(ElevatorCommands.setPos(elevatorSubsystem));
		xbox.leftBumper().onTrue(ElevatorCommands.decerementValue(elevatorSubsystem));
		xbox.rightBumper().onTrue(ElevatorCommands.incrementValue(elevatorSubsystem));

		xbox.rightTrigger().whileTrue(WristCommands.runIntake(wristSubsystem, intakeVoltage));
		xbox.rightTrigger().onFalse(WristCommands.runIntake(wristSubsystem, 0));

		xbox.leftTrigger().whileTrue(WristCommands.runOuttake(wristSubsystem, outtakeVoltage));
		xbox.leftTrigger().onFalse(WristCommands.runOuttake(wristSubsystem, 0));
		xbox.y().whileTrue(ElevatorCommands.runMotors(elevatorSubsystem));
		xbox.y().whileFalse(ElevatorCommands.stopMotors(elevatorSubsystem));
	}

	public Command getAutonomousCommand() {
		return autoChooser.get();
	}

	public void resetSimulationField() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		drive.resetOdometry(new Pose2d(3, 3, new Rotation2d()));
		SimulatedArena.getInstance().resetFieldForAuto();
	}

	public void updateSimulation() {
		if (Constants.currentMode != Constants.Mode.SIM) return;

		SimulatedArena.getInstance().simulationPeriodic();
		Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
		Logger.recordOutput(
				"FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
		Logger.recordOutput(
				"FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
	}
}
