package frc.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T3_Coral extends AutoBase {

	public T3_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		Pose2d coralPose = getPathToFeed(Paths.firstPathChooser.get())
				.getStartingHolonomicPose()
				.get();
		PathPlannerPath firstPath = Paths.firstPathChooser.get();
		PathPlannerPath secondPath = Paths.secondPathChooser.get();
		PathPlannerPath thirdPath = Paths.thirdPathChooser.get();
		addCommands(setStartPose(firstPath));
		addCommands(pathFindToPoseAndMoveElevator(coralPose, Paths.firstElevatorPosChooser.get(), elevator));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.firstElevatorPosChooser.get()));
		addCommands(followPathandMoveElevator(elevator, ElevatorPos.INTAKE, getPathToFeed(firstPath)));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(secondPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.secondElevatorPosChooser.get()));
		addCommands(followPathandMoveElevator(elevator, ElevatorPos.INTAKE, getPathToFeed(secondPath)));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(thirdPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.thirdElevatorPosChooser.get()));
	}
}
