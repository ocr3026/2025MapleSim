package frc.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class L_1Coral extends AutoBase {

	public L_1Coral(ElevatorSubsystem elevator, WristSubsystem wrist) {
		super(elevator, wrist);
		addCommands(delayStartTime());
		addCommands(Commands.runOnce(() -> SmartDashboard.putBoolean("Has timed out", false)));
		addCommands(setStartPose(Paths.FL_C1));
		addCommands(followPath(Paths.FL_C5));
		addCommands(moveElevatorAndOuttakeHomeRight(wrist, elevator, ElevatorSubsystem.firstElevatorPosChooser.get()));
		addCommands(followPath(Paths.C5_FeedL));
		addCommands(setElevatorSetpoint(ElevatorPos.INTAKE, elevator));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.FeedL_C6));
		addCommands(moveElevatorAndOuttakeHomeLeft(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(setElevatorSetpoint(ElevatorPos.INTAKE, elevator));
		addCommands(followPath(Paths.C6_FeedL));
		addCommands(moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
		addCommands(followPath(Paths.FeedL_C5));
		addCommands(moveElevatorAndOuttake(wrist, elevator, ElevatorPos.LOW));
	}
}
