package frc.robot.commands;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorCommands {
	public static final double highPOS = 0.0;
	public static final double midPOS = 0.0;
	public static final double lowPOS = 0.0;
	public static final double homePOS = 0.0;
	public static final double intakePOS = 0.0;

	public int enumID = 0;

	public static Command setPos(ElevatorSubsystem subsystem) {
		return switch (subsystem.pos) {
			case HIGH -> Commands.runOnce(() -> subsystem.setPosition(Meter.of(highPOS)));
			case MID -> Commands.runOnce(() -> subsystem.setPosition(Meter.of(midPOS)));
			case LOW -> Commands.runOnce(() -> subsystem.setPosition(Meter.of(lowPOS)));
			case INTAKE -> Commands.runOnce(() -> subsystem.setPosition(Meter.of(intakePOS)));
			case HOME -> Commands.runOnce(() -> subsystem.setPosition(Meter.of(homePOS)));
		};
	}

	public static Command incrementValue(ElevatorSubsystem subsystem) {
		return Commands.runOnce(() -> subsystem.pos = subsystem.pos.increment());
	}

	public static Command decerementValue(ElevatorSubsystem subsystem) {
		return Commands.runOnce(() -> subsystem.pos = subsystem.pos.decrement());
	}
}
