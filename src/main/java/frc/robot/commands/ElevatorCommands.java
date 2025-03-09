package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands {

	public static Distance highPOS;
	public static Distance midPOS;
	public static Distance lowPOS;
	public static Distance homePOS;
	public static Distance intakePOS;

	public static Command setPos(ElevatorSubsystem subsystem) {

		return Commands.run(() -> {
			switch (subsystem.pos) {
				case HIGH -> subsystem.setPosition((highPOS));
				case MID -> subsystem.setPosition((midPOS));
				case LOW -> subsystem.setPosition((lowPOS));
				case INTAKE -> subsystem.setPosition((intakePOS));
				case HOME -> subsystem.setPosition((homePOS));
				default -> throw new Error("NOthing selected");
			}
		});
	}

	public static Command incrementValue(ElevatorSubsystem subsystem) {
		return Commands.runOnce(() -> subsystem.pos = subsystem.pos.increment());
	}

	public static Command decerementValue(ElevatorSubsystem subsystem) {
		return Commands.runOnce(() -> subsystem.pos = subsystem.pos.decrement());
	}

	public static Command runMotors(ElevatorSubsystem subsystem) {
		return Commands.run(() -> subsystem.setSpeed(MathUtil.clamp(RobotContainer.xbox.getLeftY(), -0, 1) * 2));
	}

	public static Command stopMotors(ElevatorSubsystem subsystem) {
		return Commands.run(() -> subsystem.setSpeed(0));
	}

	// public static Command zeroEncoder(ElevatorSubsystem subsystem) {
	// 	return Commands.runOnce(() -> subsystem.)
	// }
}
