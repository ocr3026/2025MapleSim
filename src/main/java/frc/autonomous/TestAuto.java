package frc.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;

public class TestAuto extends AutoBase {

	public static Command Test1;

	@Override
	public void init() {

		Test1 = wait(.5).andThen(AutoBuilder.followPath(Paths.TEST_PATH)
				.andThen(wait(.5))
				.andThen(AutoBuilder.followPath(Paths.TEST_PATH_2)));
	}

	public static Command returnTest() {
		return setStartPose(Paths.TEST_PATH)
				.andThen(wait(.5).andThen(AutoBuilder.followPath(Paths.TEST_PATH)
						.andThen(wait(.5))
						.andThen(AutoBuilder.followPath(Paths.TEST_PATH_2))));
	}
}
