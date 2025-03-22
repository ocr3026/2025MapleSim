// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.subsystems.wrist.WristConstants.outtakeVoltage;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.autonomous.AutoBase;
import frc.autonomous.TestFiles;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.algae.AlgaeIO;
import frc.robot.subsystems.algae.AlgaeIOSpark;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.climber.ClimberIOFalcon;
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
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSpark;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Set;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.reflections.Reflections;

public class RobotContainer {
	private final ElevatorSubsystem elevatorSubsystem;
	private final WristSubsystem wristSubsystem;
	private final ClimberSubsystem climberSubsystem;
	private final AlgaeSubsystem algaeSubsystem;
	private final Drive drive;

	private final Vision vision;

	private SwerveDriveSimulation driveSimulation = null;
	UsbCamera climbCamera = CameraServer.startAutomaticCapture();

	private final CommandJoystick translationJoystick = new CommandJoystick(0);
	private final CommandJoystick rotationJoystick = new CommandJoystick(1);
	public static final CommandXboxController xbox = new CommandXboxController(2);

	private final LoggedDashboardChooser<Command> autoChooser;

	public RobotContainer() {
		switch (Constants.currentMode) {
			case SIM:
				SmartDashboard.putString("Current Pos Mode", "SIM positions");
				ElevatorCommands.highPOS = highPosConst.plus(minPosition);
				ElevatorCommands.midAlgaePOS = midAlgaePosConst.plus(minPosition);

				ElevatorCommands.midPOS = midPosConst.plus(minPosition);
				ElevatorCommands.lowAlgaePOS = lowAlgaePosConst.plus(minPosition);
				ElevatorCommands.lowPOS = lowPosConst.plus(minPosition);
				ElevatorCommands.intakePOS = intakePosConst.plus(minPosition);
				break;

			case REAL:
				SmartDashboard.putString("Current Pos Mode", "REAL positions");

				ElevatorCommands.highPOS = highPosConst;
				ElevatorCommands.midAlgaePOS = midAlgaePosConst;
				ElevatorCommands.midPOS = midPosConst;
				ElevatorCommands.lowAlgaePOS = lowAlgaePosConst;
				ElevatorCommands.lowPOS = lowPosConst;
				ElevatorCommands.intakePOS = intakePosConst;
				break;

			case REPLAY:
				SmartDashboard.putString("Current Pos Mode", "REPLAY positions");

				ElevatorCommands.highPOS = highPosConst;
				ElevatorCommands.midAlgaePOS = midAlgaePosConst;
				ElevatorCommands.midPOS = midPosConst;
				ElevatorCommands.lowAlgaePOS = lowAlgaePosConst;
				ElevatorCommands.lowPOS = lowPosConst;
				ElevatorCommands.intakePOS = intakePosConst;
				break;

			default:
				ElevatorCommands.highPOS = highPosConst;
				ElevatorCommands.midAlgaePOS = midAlgaePosConst;
				ElevatorCommands.midPOS = midPosConst;
				ElevatorCommands.lowAlgaePOS = lowAlgaePosConst;
				ElevatorCommands.lowPOS = lowPosConst;
				ElevatorCommands.intakePOS = intakePosConst;

				break;
		}
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
				algaeSubsystem = new AlgaeSubsystem(new AlgaeIOSpark());

				vision = new Vision(
						drive, new VisionIOPhotonVision(VisionConstants.camera0Name, VisionConstants.robotToCamera0));

				climberSubsystem = new ClimberSubsystem(new ClimberIOFalcon());
				break;
			case SIM:
				SmartDashboard.putString("currentRobotMode", "SIM");

				SmartDashboard.putString("currentRobotMode", "SIM");

				this.driveSimulation =
						new SwerveDriveSimulation(DriveConstants.mapleSimConfig, new Pose2d(0, 0, new Rotation2d()));

				elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
				wristSubsystem = new WristSubsystem(new WristIOSim());
				algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});

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
								driveSimulation::getSimulatedDriveTrainPose));

				climberSubsystem = new ClimberSubsystem(new ClimberIOFalcon());
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
				algaeSubsystem = new AlgaeSubsystem(new AlgaeIO() {});
				vision = new Vision(drive, new VisionIO() {}, new VisionIO() {});

				climberSubsystem = new ClimberSubsystem(new ClimberIOFalcon());
				break;
		}
		// Test01Auto test01Auto = new Test01Auto(elevatorSubsystem, wristSubsystem);
		// Test02Auto test02Auto = new Test02Auto();
		// Test03Auto test03Auto = new Test03Auto();
		autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
		// autoChooser.addOption("Far Left to C3 2 Coral", new FL_C3_2C(elevatorSubsystem, wristSubsystem));

		// Paths.initPaths();
		// Paths.initAutoFactory();

		Reflections reflection = new Reflections("frc.autonomous");
		Set<Class<? extends AutoBase>> autoClasses = reflection.getSubTypesOf(AutoBase.class);

		for (Class<? extends AutoBase> autoClass : autoClasses) {
			try {

				Constructor<? extends AutoBase> constructor =
						autoClass.getConstructor(ElevatorSubsystem.class, WristSubsystem.class);
				SequentialCommandGroup command;
				command = constructor.newInstance(elevatorSubsystem, wristSubsystem);
				autoChooser.addOption(autoClass.getSimpleName() + " Auto", command);

			} catch (NoSuchMethodException
					| SecurityException
					| InstantiationException
					| IllegalAccessException
					| IllegalArgumentException
					| InvocationTargetException e) {
				// TODO Auto-generated catch block
				DriverStation.reportError(e.getMessage(), e.getStackTrace());
				Constructor<? extends AutoBase> constructor;
				try {
					constructor = autoClass.getConstructor(WristSubsystem.class, ElevatorSubsystem.class);
					SequentialCommandGroup command;
					command = constructor.newInstance(wristSubsystem, elevatorSubsystem);
					autoChooser.addOption(autoClass.getSimpleName() + " Auto", command);
				} catch (NoSuchMethodException
						| SecurityException
						| InstantiationException
						| IllegalAccessException
						| IllegalArgumentException
						| InvocationTargetException e1) {
					// TODO Auto-generated catch block
					DriverStation.reportError(e.getMessage(), e.getStackTrace());
				}
			}
		}
		autoChooser.addOption("TestFiles", new TestFiles(elevatorSubsystem, wristSubsystem));

		/*
		autoChooser.addOption("Drive Wheel Radius Characterization6", DriveCommands.wheelRadiusCharacterization(drive));
		autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption(
				"Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
		autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
		autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
		*/

		configureBindings();
	}

	private void configureBindings() {
		drive.setDefaultCommand(DriveCommands.joystickDrive(
				drive,
				() -> -translationJoystick.getY(),
				() -> -translationJoystick.getX(),
				() -> -rotationJoystick.getX()));

		translationJoystick.button(11).onTrue(Commands.runOnce(drive::stopWithX, drive));

		final Runnable resetGyro = Constants.currentMode == Constants.Mode.SIM
				? () -> drive.resetOdometry(
						driveSimulation
								.getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
				: () -> drive.resetOdometry(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));

		translationJoystick.button(12).onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));
		rotationJoystick.button(3).whileTrue(DriveCommands.lookAtCoral(drive, vision));

		// climberSubsystem.setDefaultCommand(ClimberCommands.moveClimber(climberSubsystem, xbox.getLeftY()));
		//	xbox.y().whileTrue(ClimberCommands.autoPositionClimber(climberSubsystem, 45)
		//	.andThen(ClimberCommands.autoPositionClimber(climberSubsystem, 135)));
		xbox.a().whileTrue(ElevatorCommands.setPos(elevatorSubsystem));
		xbox.leftBumper().onTrue(ElevatorCommands.decerementValue(elevatorSubsystem));
		xbox.rightBumper().onTrue(ElevatorCommands.incrementValue(elevatorSubsystem));

		xbox.button(8).and(translationJoystick.button(3)).whileTrue(ClimberCommands.runTrapdoor(climberSubsystem));

		// R3

		xbox.button(9).whileTrue(ClimberCommands.moveClimber(climberSubsystem));

		// outtake for home/t1
		xbox.b().whileTrue(WristCommands.runIntake(wristSubsystem, -3, -0.5));
		xbox.b().onFalse(WristCommands.runOuttake(wristSubsystem, 0, 0));

		xbox.x().whileTrue(algaeSubsystem.runAlgaeManipulator());

		xbox.rightTrigger().whileTrue(WristCommands.runOuttake(wristSubsystem, outtakeVoltage, outtakeVoltage));
		xbox.rightTrigger().onFalse(WristCommands.runOuttake(wristSubsystem, 0, 0));

		xbox.y().whileTrue(WristCommands.runOuttake(wristSubsystem, 2, 2));
		xbox.y().onFalse(WristCommands.runOuttake(wristSubsystem, 0, 0));

		// xbox.leftTrigger().whileTrue(WristCommands.runOuttake(wristSubsystem, outtakeVoltage, outtakeVoltage));
		// xbox.leftTrigger().onFalse(WristCommands.runIntake(wristSubsystem, 0, 0));

		xbox.leftTrigger()
				.whileTrue(AutoBase.moveElevatorAndIntake(wristSubsystem, elevatorSubsystem, ElevatorPos.INTAKE));
		xbox.leftTrigger().onTrue(AutoBase.setElevatorSetpoint(ElevatorPos.INTAKE, elevatorSubsystem));
		xbox.leftTrigger().onFalse(WristCommands.runOuttake(wristSubsystem, 0, 0));

		// xbox.y().whileTrue(ElevatorCommands.runMotors(elevatorSubsystem));
		// xbox.y().onFalse(ElevatorCommands.stopMotors(elevatorSubsystem));
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

	public void onTeamSwitch() {
		Reflections reflection = new Reflections("frc.autonomous");
		Set<Class<? extends AutoBase>> autoClasses = reflection.getSubTypesOf(AutoBase.class);

		for (Class<? extends AutoBase> autoClass : autoClasses) {
			try {

				Constructor<? extends AutoBase> constructor =
						autoClass.getConstructor(ElevatorSubsystem.class, WristSubsystem.class);
				SequentialCommandGroup command;
				command = constructor.newInstance(elevatorSubsystem, wristSubsystem);
				autoChooser.addOption(autoClass.getSimpleName() + " Auto", command);

			} catch (NoSuchMethodException
					| SecurityException
					| InstantiationException
					| IllegalAccessException
					| IllegalArgumentException
					| InvocationTargetException e) {
				Constructor<? extends AutoBase> constructor;
				try {
					constructor = autoClass.getConstructor(WristSubsystem.class, ElevatorSubsystem.class);
					SequentialCommandGroup command;
					command = constructor.newInstance(wristSubsystem, elevatorSubsystem);
					autoChooser.addOption(autoClass.getSimpleName() + " Auto", command);
				} catch (NoSuchMethodException
						| SecurityException
						| InstantiationException
						| IllegalAccessException
						| IllegalArgumentException
						| InvocationTargetException e1) {
					// TODO Auto-generated catch block
					DriverStation.reportError(e.getMessage(), e.getStackTrace());
				}
			}
		}
	}
}
