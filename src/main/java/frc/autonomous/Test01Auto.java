package frc.autonomous;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class Test01Auto extends AutoBase {

	public Test01Auto(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.CORAL_FL_PATH));
		addCommands(followPath(Paths.CORAL_FL_PATH));
		addCommands(Commands.runOnce(() -> setElevatorSetpoint(ElevatorPos.HIGH, elevatorSubsystem)));
		addCommands(moveElevatorAndOuttake(wristSubsystem, elevatorSubsystem, ElevatorPos.HIGH));
		addCommands(wait(0.5));
		addCommands(followPath(Paths.CC_TO_FEED));
		addCommands(moveElevatorAndOuttake(wristSubsystem, elevatorSubsystem, ElevatorPos.INTAKE));
		addCommands(wait(1.0));
		addCommands(followPath(Paths.FEED_TO_CC));
		addCommands(moveElevatorAndOuttake(wristSubsystem, elevatorSubsystem, ElevatorPos.MID));
	}

	@Override
	public void init() {}
}
