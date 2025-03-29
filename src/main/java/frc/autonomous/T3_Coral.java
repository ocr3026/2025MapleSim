package frc.autonomous;

import static frc.autonomous.AutoBase.Paths.secondElevatorPosChooser;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class T3_Coral extends AutoBase {

	public T3_Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		PathPlannerPath firstPath = Paths.firstPathChooser.get();
		PathPlannerPath secondPath = Paths.secondPathChooser.get();
		PathPlannerPath thirdPath = Paths.thirdPathChooser.get();
		addCommands(setStartPose(firstPath));
		addCommands(followPath(firstPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.firstElevatorPosChooser.get()));
		addCommands(followPathandMoveElevator(elevator, ElevatorPos.INTAKE, getPathToFeed(firstPath)));
		addCommands(wait(0.5));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPathandMoveElevator(elevator, Paths.secondElevatorPosChooser.get(), secondPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.secondElevatorPosChooser.get()));
		addCommands(followPathandMoveElevator(elevator, ElevatorPos.INTAKE, getPathToFeed(secondPath)));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPathandMoveElevator(elevator, Paths.thirdElevatorPosChooser.get(), thirdPath));
		addCommands(moveElevatorAndOuttake(wrist, elevator, Paths.thirdElevatorPosChooser.get()));
	}
}
