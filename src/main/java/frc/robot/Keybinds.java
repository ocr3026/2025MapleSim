package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Keybinds {
	public static final Trigger resetGyroTrigger = RobotContainer.translationJoystick.button(12);
	public static final Trigger lookAtCoralTrigger = RobotContainer.translationJoystick.button(3);

	public static final Trigger moveElevatorTrigger = RobotContainer.xbox.a();
	public static final Trigger decrementElevatorEnumTrigger = RobotContainer.xbox.leftBumper();
	public static final Trigger incrementElevatorEnumTrigger = RobotContainer.xbox.rightBumper();

	public static final Trigger runIntakeWithSensorTrigger = RobotContainer.xbox.b();
	public static final Trigger runIntakeTrigger = RobotContainer.xbox.y();
	public static final Trigger runOuttakeTrigger = RobotContainer.xbox.rightTrigger();
	public static final Trigger runIntakeAndElevatorTrigger = RobotContainer.xbox.leftTrigger();

	public static final Trigger runAlgaeManipulatorTrigger = RobotContainer.xbox.x();

	public static final Trigger moveClimberTrigger = RobotContainer.xbox.button(9);
	public static final Trigger xboxTrapdoorTrigger = RobotContainer.xbox.button(8);
	public static final Trigger joystickTrapdoorTrigger = RobotContainer.translationJoystick.button(3);
}
