package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public final class DriveConstants {
	public static final LinearVelocity maxSpeed = MetersPerSecond.of(5.0);
	public static final double odometryFrequency = 100; // Hertz
	public static final Distance trackWidth = Inches.of(24.75);
	public static final Distance wheelBase = Inches.of(24.75);
	public static final Distance driveBaseRadius =
			Meters.of(Math.hypot(trackWidth.div(2).in(Meters), wheelBase.div(2).in(Meters)));
	public static final Translation2d[] moduleTranslations = new Translation2d[] {
		new Translation2d(trackWidth.div(2).in(Meters), wheelBase.div(2).in(Meters)),
		new Translation2d(
				trackWidth.div(2).in(Meters), wheelBase.div(2).unaryMinus().in(Meters)),
		new Translation2d(
				trackWidth.div(2).unaryMinus().in(Meters), wheelBase.div(2).in(Meters)),
		new Translation2d(
				trackWidth.div(2).unaryMinus().in(Meters),
				wheelBase.div(2).unaryMinus().in(Meters))
	};


	public static final int frontLeftDriveID = 10;
	public static final int frontLeftTurnID = 11;
	public static final int frontLeftEncoderID = 12;
	
	public static final int frontRightDriveID = 15;
	public static final int frontRightTurnID = 16;
	public static final int frontRightEncoderID = 17;
	
	public static final int rearLeftDriveID = 20;
	public static final int rearLeftTurnID = 21;
	public static final int rearLeftEncoderID = 22;

	public static final int rearRightDriveID = 25;
	public static final int rearRightTurnID = 26;
	public static final int rearRightEncoderID = 27;


	public static final int driveMotorCurrentLimit = 60;
	public static final Distance wheelRadius = Inches.of(2);
	public static final double driveMotorReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
	public static final DCMotor driveGearbox = DCMotor.getNEO(1);

	public static final double driveEncoderPositionFactor =
			2 * Math.PI * wheelRadius.in(Meters) / (driveMotorReduction);
	public static final double driveEncoderVelocityFactor =
			2 * Math.PI * wheelRadius.in(Meters) / (driveMotorReduction);

	public static final double driveKp = 0, driveKd = 0; // FB
	public static final double driveKs = 0.14, driveKv = 2.35; // FF
	public static final double driveSimP = 0.05, driveSimD = 0; // FB
	public static final double driveSimKs = 0, driveSimKv = 2.35; // FF

	public static final boolean turnInverted = false;
	public static final int turnMotorCurrentLimit = 30;
	public static final double turnMotorReduction = 150.0 / 7.0;
	public static final DCMotor turnGearbox = DCMotor.getNEO(1);

	public static final double turnEncoderPositionFactor = 1.0 / (turnMotorReduction);
	public static final double turnEncoderVelocityFactor = 1.0 / (turnMotorReduction * 60);

	// TODO: make sure these line up with onboard values
	public static final SensorDirectionValue encoderDirection = SensorDirectionValue.Clockwise_Positive;
	public static final double absoluteSensorDiscontinuityPoint = 0.5;
	public static final Rotation2d frontLeftMagnetOffset = Rotation2d.fromRotations(-0.202392578125);
	public static final Rotation2d frontRightMagnetOffset = Rotation2d.fromRotations(-0.0361328125);
	public static final Rotation2d rearLeftMagnetOffset = Rotation2d.fromRotations(0.195068359375);
	public static final Rotation2d rearRightMagnetOffset = Rotation2d.fromRotations(0.356689453125);
	public static final double turnKp = 5, turnKd = 0;
	public static final double turnSimP = 8, turnSimD = 0;
	public static final Rotation2d turnPIDMinInput = Rotation2d.fromDegrees(-180),
			turnPIDMaxInput = Rotation2d.fromDegrees(180);

	// PathPlanner configuration
	public static final Mass robotMass = Pounds.of(100);
	public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(6.883);
	public static final double wheelCOF = 1.2;
	public static final RobotConfig ppConfig = new RobotConfig(
			robotMass.in(Kilogram),
			robotMOI.in(KilogramSquareMeters),
			new ModuleConfig(
					wheelRadius,
					maxSpeed,
					wheelCOF,
					driveGearbox.withReduction(driveMotorReduction),
					Amps.of(driveMotorCurrentLimit),
					1),
			moduleTranslations);

	public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
			.withCustomModuleTranslations(moduleTranslations)
			.withRobotMass(Kilogram.of(robotMass.in(Kilogram)))
			.withGyro(COTS.ofNav2X())
			.withSwerveModule(new SwerveModuleSimulationConfig(
					driveGearbox,
					turnGearbox,
					driveMotorReduction,
					turnMotorReduction,
					Volts.of(0.1),
					Volts.of(0.1),
					wheelRadius,
					KilogramSquareMeters.of(0.02),
					wheelCOF));
}
