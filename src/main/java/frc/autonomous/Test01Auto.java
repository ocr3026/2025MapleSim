package frc.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class Test01Auto extends AutoBase {
	public static Command finalCommand;
	public static ElevatorSubsystem elevatorSubsystem;
	public static WristSubsystem wristSubsystem;

	public Test01Auto(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
		Test01Auto.elevatorSubsystem = elevatorSubsystem;
		Test01Auto.wristSubsystem = wristSubsystem;
	}

	@Override
	public void init() {
		addCommands(delayStartTime());
		addCommands(followPath(Paths.TEST_PATH_2));
		addCommands(wristIntake(wristSubsystem, elevatorSubsystem));
		addCommands(followPath(Paths.TEST_PATH));
	}
}
