package frc.robot.subsystems.drive;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
	public static final double maxSpeedMetersPerSec = 5.0;
	public static final double odemetryFrequency = 100; // Hz
	public static final double trackWidth = Units.inchesToMeters(26.5);
	public static final double wheelBase = Units.inchesToMeters(26.5);
	public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
	public static final Translation2d[] moduleTranslations = new Translation2d[] {
		new Translation2d(trackWidth / 2.0, wheelBase / 2),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
	};

	public static final int frontLeftDriveID = 2;
	public static final int frontRightDriveID = 4;
	public static final int rearLeftDriveID = 6;
	public static final int rearRightDriveID = 8;

	public static final int frontLeftTurnID = 3;
	public static final int frontRightTurnID = 5;
	public static final int rearLeftTurnID = 7;
	public static final int rearRightTurnID = 9;

	public static final int frontLeftEncoderID = 10;
	public static final int frontRightEncoderID = 11;
	public static final int rearLeftEncoderID = 12;
	public static final int rearRightEncoderID = 13;

	public static final int driveMotorCurrentLimit = 60;
	public static final double wheelRadiusMeters = Units.inchesToMeters(4);
	public static final double driveMotorReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
	public static final DCMotor driveGearbox = DCMotor.getNEO(1);

	public static final double driveEncoderPositionFactor = 2.0 * Math.PI / driveMotorReduction; // Rotor Rotation -> Wheel Radians
	public static final double driveEncoderVelocityFactor = (2.0 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM -> Wheel Rad/Sec

	public static final double driveKp = 0, driveKd = 0; // FB
	public static final double driveKs = 0.14, driveKv = 2.35, driveKa = 0.36; // FF
	public static final double driveSimP = 0.05, driveSimD = 0; // FB
	public static final double driveSimKs = 0, driveSimKv = 2.35, driveSimKa = 0.36; // FF

	public static final boolean turnInverted = false;
	public static final int turnMotorCurrentLimit = 20;
	public static final double turnMotorReduction = 150.0 / 7.0;
	public static final DCMotor turnGearbox = DCMotor.getNEO(1);

	// TODO: make sure these line up with onboard values
	public static final SensorDirectionValue encoderDirection = SensorDirectionValue.CounterClockwise_Positive;
	public static final double sensorAbsoluteDiscontinuityPoint = 0.5;
	public static final double frontLeftMagnetOffset = 0;
    public static final double frontRightMagnetOffset = 0;
    public static final double backLeftMagnetOffset = 0;
    public static final double backRightMagnetOffset = 0;
}
