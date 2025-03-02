package frc.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristSubsystem;

public class TestAuto extends AutoBase {

	public static Command Test1;

	@Override
	public void init() {
		Test1 = wait(.5).andThen(AutoBuilder.followPath(Paths.TEST_PATH)
				.andThen(wait(.5))
				.andThen(AutoBuilder.followPath(Paths.TEST_PATH_2)));
	}

	public static Command returnTest(ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
		return delayStartTime()
				.andThen(setStartPose(Paths.TEST_PATH))
				.andThen(wait(.5).andThen(AutoBuilder.followPath(Paths.TEST_PATH)
						.andThen(setElevatorPosition(elevatorSubsystem, ElevatorPos.HIGH))
						.andThen(moveElevatorAndIntake(wristSubsystem, elevatorSubsystem))
						.andThen(wait(.5))
						.andThen(AutoBuilder.followPath(Paths.TEST_PATH_2))));
	}
}
