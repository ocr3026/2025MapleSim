package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands {
	public static final Distance highPOS = Meters.of(0.590);
	public static final Distance midPOS = Meters.of(0.269);
	public static final Distance lowPOS = Inches.of(0);
	public static final Distance homePOS = Inches.of(0.0);
	public static final Distance intakePOS = Inches.of(0);

	public static Command setPos(ElevatorSubsystem subsystem) {
		// return Commands.runOnce(() -> subsystem.setPosition(Inches.of(lowPOS)));

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
		return Commands.run(() -> subsystem.setSpeed(MathUtil.clamp(RobotContainer.xbox.getLeftY(), -1, 1) * 12));
	}

	public static Command stopMotors(ElevatorSubsystem subsystem) {
		return Commands.run(() -> subsystem.setSpeed(0));
	}

	// public static Command zeroEncoder(ElevatorSubsystem subsystem) {
	// 	return Commands.runOnce(() -> subsystem.)
	// }
}
