package frc.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T2_Coral extends AutoBase {
	public T2_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		Pose2d coralPose = getPathToFeed(Paths.firstPathChooser.get())
				.getStartingHolonomicPose()
				.get();

		addCommands(setStartPose(Paths.firstPathChooser.get()));

		addCommands(pathFindToPose(coralPose));
		// addCommands(followPath(Paths.firstPathChooser.get()));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.firstElevatorPosChooser.get()));
		addCommands(
				followPathandMoveElevator(elevator, ElevatorPos.INTAKE, getPathToFeed(Paths.firstPathChooser.get())));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.secondPathChooser.get()));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.secondElevatorPosChooser.get()));
	}
}
// 5.34
