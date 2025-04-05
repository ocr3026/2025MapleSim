package frc.robot.subsystems.wrist;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class WristSubsystem extends SubsystemBase {
	private final WristIO io;
	private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

	private static final double debounceTime = 0.01;
	private Notifier notifier = new Notifier(this::debounce);
	public static AtomicBoolean getCoralInputBool = new AtomicBoolean(false);
	private static Debouncer ultrasonicDebouncer = new Debouncer(debounceTime);

	public static LoggedMechanismLigament2d wristLigament;

	public WristSubsystem(WristIO wristIO) {
		this.io = wristIO;
		notifier.setName("SensorDebounceThread");
		notifier.startPeriodic(debounceTime / 3);
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
		SmartDashboard.putBoolean("has coral or summ", getCoralInputBool.get());
	}

	public void debounce() {
		getCoralInputBool.set(ultrasonicDebouncer.calculate(getCoralInput()));
	}
}
