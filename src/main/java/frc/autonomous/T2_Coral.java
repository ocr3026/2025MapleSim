package frc.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T2_Coral extends AutoBase {
	public T2_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		Pose2d coralPose = getPathToFeed(Paths.firstPathChooser.get())
				.getStartingHolonomicPose()
				.get();
		Command startingCommand = pathFindToPose(coralPose);

		addCommands(setStartPose(Paths.firstPathChooser.get()));
		// addCommands(AutoBuilder.pathfindToPose(
		// 		FlippingUtil.flipFieldPose(Paths.paths
		// 				.get("C4 to FeedL")
		// 				.getStartingHolonomicPose()
		// 				.get()),
		// 		DriveConstants.autoConstraints));
		addCommands(startingCommand);
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
