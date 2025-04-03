package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands {

	public static Distance highPOS;
	public static Distance midAlgaePOS;
	public static Distance midPOS;
	public static Distance lowAlgaePOS;
	public static Distance lowPOS;
	public static Distance intakePOS;

	/**
	 * @param subsystem
	 * @return Command
	 */
	public static Command setPos(ElevatorSubsystem subsystem) {
		if (Constants.currentMode != Constants.Mode.SIM) {
			return Commands.run(() -> {
				switch (subsystem.pos) {
					case HIGH -> subsystem.setPosition((highPOS));
					case MIDALGAE -> subsystem.setPosition(midAlgaePOS);
					case MID -> subsystem.setPosition((midPOS));
					case LOWALGAE -> subsystem.setPosition(lowAlgaePOS);
					case LOW -> subsystem.setPosition((lowPOS));
					case INTAKE -> subsystem.setPosition((intakePOS));
					default -> throw new Error("NOthing selected");
				}
			});

		} else {
			return Commands.run(() -> {
				switch (subsystem.pos) {
					case HIGH -> subsystem.setPosition((highPOS));
					case MIDALGAE -> subsystem.setPosition(midAlgaePOS);
					case MID -> subsystem.setPosition((midPOS));
					case LOWALGAE -> subsystem.setPosition(lowAlgaePOS);
					case LOW -> subsystem.setPosition((lowPOS));
					case INTAKE -> subsystem.setPosition((intakePOS));
					default -> throw new Error("NOthing selected");
				}
			});
		}
	}

	public static Command incrementValue(ElevatorSubsystem subsystem) {
		return Commands.runOnce(() -> subsystem.pos = subsystem.pos.increment());
	}

	public static Command decerementValue(ElevatorSubsystem subsystem) {
		return Commands.runOnce(() -> subsystem.pos = subsystem.pos.decrement());
	}

	public static Command runMotors(ElevatorSubsystem subsystem) {
		return Commands.run(() -> subsystem.setSpeed(MathUtil.clamp(RobotContainer.xbox.getLeftY(), -0.05, .05)));
	}

	public static Command setSpeed(ElevatorSubsystem subsystem, double speed) {
		return Commands.run(() -> subsystem.setSpeed(speed));
	}

	public static Command stopMotors(ElevatorSubsystem subsystem) {
		return Commands.run(() -> subsystem.setSpeed(0));
	}

	public static Command setMotorPos(ElevatorSubsystem subsystem) {
		return Commands.runOnce(() -> subsystem.zeroElevator(), subsystem);
	}

	// public static Command zeroEncoder(ElevatorSubsystem subsystem) {
	// 	return Commands.runOnce(() -> subsystem.)
	// }
}
