package frc.robot.subsystems.wrist;

import edu.wpi.first.math.system.plant.DCMotor;

public class WristConstants {
	public static final int currentLimit = 40;
	public static final int leadMotorID = 50;
	public static final int followMotorID = 51;

	public static final DCMotor gearbox = DCMotor.getNeo550(2);

	public static final double intakeVoltage = -3;
	public static final double outtakeVoltage = -6;
	public static final double slowOuttakeVoltage = -2;
	public static final double slowIntakeVoltage = 0;
}
