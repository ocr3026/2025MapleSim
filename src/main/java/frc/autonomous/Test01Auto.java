package frc.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class Test01Auto extends AutoBase {

	public Test01Auto(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
		addCommands(delayStartTime());
		addCommands(setStartPose(Paths.TEST_PATH));
		addCommands(followPath(Paths.TEST_PATH));
		addCommands(wristIntake(wristSubsystem, elevatorSubsystem));
		addCommands(followPath(Paths.TEST_PATH));
	}

	@Override
	public void init() {
		
	}
}
