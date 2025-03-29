package frc.robot.commands;

import static frc.robot.subsystems.wrist.WristConstants.slowIntakeVoltage;
import static frc.robot.subsystems.wrist.WristConstants.slowOuttakeVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.WristSubsystem;

public class WristCommands {
	static boolean seenCoral = false;
	static boolean coralInWrist = false;
	static boolean coralInPosition = false;

	/**
	 * @param subsystem
	 * @param leadVoltage
	 * @param followVoltage
	 * @return Command
	 */
	public static Command runOuttake(WristSubsystem subsystem, double leadVoltage, double followVoltage) {
		return Commands.run(() -> subsystem.setVoltage(leadVoltage, followVoltage));
	}

	public static Command AlgaeOuttake(WristSubsystem subsystem) {
		return Commands.runEnd(
				() -> {
					subsystem.turnRotations(10);
				},
				() -> {
					subsystem.setVoltage(0, 0);
				});
	}

	public static Command runIntake(WristSubsystem subsystem, double leadVoltage, double followVoltage) {
		return Commands.startRun(
				() -> {
					seenCoral = false;
					coralInWrist = false;
					coralInPosition = false;
				},
				() -> {
					if (coralInPosition) {
						return;
					}

					if (seenCoral) {
						if (WristSubsystem.getCoralInputBool && !coralInWrist) {
							subsystem.setVoltage(slowOuttakeVoltage, slowOuttakeVoltage);
						} else {
							coralInWrist = true;
							if (WristSubsystem.getCoralInputBool) {
								subsystem.setVoltage(0, 0);
								coralInPosition = true;
							} else {
								subsystem.setVoltage(slowIntakeVoltage, slowIntakeVoltage);
							}
						}
					} else {
						if (WristSubsystem.getCoralInputBool) {
							seenCoral = true;
						}
						subsystem.setVoltage(leadVoltage, followVoltage);
					}
				},
				subsystem);
	}
}
