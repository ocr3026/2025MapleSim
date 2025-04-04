package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;

public class WristConstants {
	public static final int currentLimit = 50;
	public static final int leadMotorID = 50;
	public static final int followMotorID = 51;

	public static final DCMotor gearbox = DCMotor.getNeo550(2);

	public static final double intakeVoltage = -3;
	public static final double intakeFollowVoltage = -3;
	public static final double outtakeVoltage = -4.0; // TODO:
	public static final double outtakeVoltageHigh = -5.0;
	public static final double slowOuttakeVoltage = -2;
	public static final double slowIntakeVoltage = 1;
}
