package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {
	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

	public static final ElevatorPos pos = ElevatorPos.HOME;

	public static enum Mode {
		REAL,
		SIM,
		REPLAY
	}

	public static enum ElevatorPos {
		HIGH(4),
		MID(3),
		LOW(2),
		HOME(0),
		INTAKE(1);

		private final int id;
		ElevatorPos(int id) {this.id = id;}
		public int getValue() {return id;}
	}


	/*
	 * FOR SELECTING THE ELEVATOR POSITION MAKE A HOTBAR WITH TRIGGERS - BUTTON A TO SELECT THE POS
	 * INSTANT/SELECT
	 * 
	 * 
	 * 
	 */
}
