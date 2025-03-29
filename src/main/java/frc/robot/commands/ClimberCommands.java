package frc.robot.commands;

import static frc.robot.RobotContainer.xbox;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommands {

	public ClimberCommands() {}

	/**
	 * @param climberSubsystem
	 * @return Command
	 */
	public static Command moveClimber(ClimberSubsystem climberSubsystem) {

		return Commands.runEnd(
				() -> {
					if (xbox.getRightY() <= -0.1) {

						climberSubsystem.setSpeed(-1);
					} else if (xbox.getRightY() >= 0.1) {
						climberSubsystem.setSpeed(1);
					} else {
						climberSubsystem.setSpeed(0);
					}
				},
				() -> {
					climberSubsystem.setSpeed(0);
				},
				climberSubsystem);
	}

	public static Command runTrapdoor(ClimberSubsystem climberSubsystem) {
		return Commands.runEnd(
				() -> {
					climberSubsystem.runTrapdoor();
				},
				() -> {
					climberSubsystem.stopTrapdoor();
				},
				climberSubsystem);
	}
}
