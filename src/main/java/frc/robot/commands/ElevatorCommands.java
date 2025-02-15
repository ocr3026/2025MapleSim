package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.measure.Distance.*;
import static frc.robot.Constants.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands {
	public static final Distance highPOS = Inches.of(30.0 + 38.0);
	public static final Distance midPOS = Inches.of(25.0 + 38.0);
	public static final Distance lowPOS = Inches.of(15.0 + 38.0);
	public static final Distance homePOS = Inches.of(0.0 + 38.0);
	public static final Distance intakePOS = Inches.of(20.0 + 38.0);

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
		return Commands.run(() -> subsystem.setSpeed(RobotContainer.xbox.getLeftY()));
	}

	public static Command stopMotors(ElevatorSubsystem subsystem) {
		return Commands.run(() -> subsystem.setSpeed(0));
	}
}
