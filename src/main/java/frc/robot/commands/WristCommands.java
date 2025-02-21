package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.WristSubsystem;

public class WristCommands {

	public static Command runIntake(WristSubsystem subsystem, double voltage) {
		return Commands.run(() -> subsystem.setVoltage(voltage));
	}

	public static Command runOuttake(WristSubsystem subsystem, double voltage) {
		return Commands.run(() -> subsystem.setVoltage(voltage));
	}
}
