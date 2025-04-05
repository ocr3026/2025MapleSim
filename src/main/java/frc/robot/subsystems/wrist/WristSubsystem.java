package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class WristSubsystem extends SubsystemBase {
	private final WristIO io;
	private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
	public static boolean getCoralInputBool = false;
	// private static Debouncer ultrasonicDebouncer = new Debouncer(0.03);

	public static LoggedMechanismLigament2d wristLigament;

	public WristSubsystem(WristIO wristIO) {
		this.io = wristIO;
	}

	/**
	 * @param leadVoltage
	 * @param followVoltage
	 */
	public void setVoltage(double leadVoltage, double followVoltage) {
		io.setVoltage(leadVoltage, followVoltage);
	}

	public void turnRotations(double rotations) {

		double goalRotations = io.getRotations() + rotations;
		if (io.getRotations() != goalRotations) {
			io.setVoltage(-1, -1);
		} else {
			io.setVoltage(0, 0);
		}
	}

	/** @return boolean */
	public boolean getCoralInput() {
		return io.getCoralInput();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Wrist", inputs);
		// getCoralInputBool = ultrasonicDebouncer.calculate(getCoralInput());
		getCoralInputBool = getCoralInput();
		SmartDashboard.putBoolean("has coral or summ", getCoralInputBool);
	}
}
