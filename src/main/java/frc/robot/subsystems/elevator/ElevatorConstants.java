package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;

public final class ElevatorConstants {
	public static final int leadMotorID = 30;
	public static final int followMotorID = 31;

	public static final Distance elevatorHeight = Inches.of(38);
	public static final Distance elevatorWidth = Inches.of(19.5);

	public static final Distance softwareLimit = Meters.of(0.660);

	public static final Distance minPosition = Inches.of(38),
			maxPosition = Meters.of(softwareLimit.in(Meter) + minPosition.in(Meter));

	public static final DCMotor gearbox = DCMotor.getNEO(2);
	public static final double elevatorReduction = 6;
	public static final Distance spoolDiameter = Inches.of(1.75);

	public static final double encoderPositionFactor =
			Math.PI * spoolDiameter.in(Meter) / elevatorReduction; // Rotations to meters
	public static final double encoderVelocityFactor =
			Math.PI * spoolDiameter.in(Meter) / (elevatorReduction * 60); // RPM to m/s

	public static final double kP = 10, kI = 0.1, kD = 0, kS = 0, kV = 1.75, kA = 0.15, kG = 1.40;
	public static final double simkP = 5, simkD = 0, simkS = 0, simkV = 1.75, simkA = 0.15, simkG = 1.26;

	public static final int currentLimit = 50;

	public static final Distance highPosConst = Meters.of(0.660),
			midAlgaePosConst = Meters.of(0.3507),
			midPosConst = Meters.of(0.3507),
			lowAlgaePosConst =Meters.of(.116),
			lowPosConst = Meters.of(0.116),
			homePosConst = Meters.of(0.0),
			intakePosConst = Meters.of(0.0);
	// https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=93&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A2%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.75%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A30%2C%22u%22%3A%22in%22%7D
	// https://www.reca.lc/linear?angle=%7B%22s%22%3A90%2C%22u%22%3A%22deg%22%7D&currentLimit=%7B%22s%22%3A50%2C%22u%22%3A%22A%22%7D&efficiency=93&limitAcceleration=0&limitDeceleration=0&limitVelocity=0&limitedAcceleration=%7B%22s%22%3A400%2C%22u%22%3A%22in%2Fs2%22%7D&limitedDeceleration=%7B%22s%22%3A50%2C%22u%22%3A%22in%2Fs2%22%7D&limitedVelocity=%7B%22s%22%3A10%2C%22u%22%3A%22in%2Fs%22%7D&load=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A2%2C%22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A1.75%2C%22u%22%3A%22in%22%7D&travelDistance=%7B%22s%22%3A30%2C%22u%22%3A%22in%22%7D
}
