package frc.robot;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

	public static enum Mode {
		REAL,
		SIM,
		REPLAY
	}

	public static final Frequency logFrequency = Hertz.of(50);

	/*
	 * FOR SELECTING THE ELEVATOR POSITION MAKE A HOTBAR WITH TRIGGERS - BUTTON A TO SELECT THE POS
	 * INSTANT/SELECT
	 *
	 *
	 *
	 */
}
