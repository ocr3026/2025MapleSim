package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public final class ClimberConstants {
	public static final int climbMotorID = 40;
	public static final int trapdoorMotorID = 41;

	public static final Distance climberLength = Inches.of(14.5);

	public static final Angle minPosition = Degrees.of(25), maxPosition = Degrees.of(135);

	public static final DCMotor gearbox = DCMotor.getNEO(1);
	public static final double climberReduction = 1000;

	public static final double encoderPositionFactor = 360 / climberReduction;
	public static final double encoderVelocityFactor = 360 / (climberReduction * 60);

	public static final double kP = 1, kD = 0, kS = 0, kV = 1.75, kA = 0.15, kG = 1.26;
	public static final double simkP = kP * 12,
			simkD = kD * 12,
			simkS = kS * 12,
			simkV = kV * 12,
			simkA = kA * 12,
			simkG = kG * 12;
	public static final double currentLimit = 50;

	// https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=93&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A2%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.75%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A30%2C%22u%22%3A%22in%22%7D
}
