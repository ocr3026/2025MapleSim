package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

	public static enum Mode {
		REAL,
		SIM,
		REPLAY
	}

	/*
	 * FOR SELECTING THE ELEVATOR POSITION MAKE A HOTBAR WITH TRIGGERS - BUTTON A TO SELECT THE POS
	 * INSTANT/SELECT
	 *
	 *
	 *
	 */
}
