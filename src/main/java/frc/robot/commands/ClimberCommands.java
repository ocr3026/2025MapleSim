package frc.robot.commands;

import static frc.robot.RobotContainer.xbox;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimberCommands {

	public ClimberCommands() {}
	;

	public static Command autoPositionClimber(ClimberSubsystem climberSubsystem, double angle) {
		return new FunctionalCommand(
				() -> {
					climberSubsystem.setSpeed(0);
				},
				() -> {
					climberSubsystem.setPosition(Angle.ofBaseUnits(angle, null));
				},
				(interupted) -> {
					climberSubsystem.setSpeed(0);
				},
				() -> {
					return MathUtil.isNear(angle, climberSubsystem.getPositionInDegrees(), 5);
				},
				climberSubsystem);
	}

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
