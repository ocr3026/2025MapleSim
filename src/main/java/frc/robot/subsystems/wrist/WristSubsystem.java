package frc.robot.subsystems.wrist;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class WristSubsystem extends SubsystemBase {
	private final WristIO io;
	private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
	public static boolean getCoralInputBool = false;
	private static Debouncer ultrasonicDebouncer = new Debouncer(0.1);

	public static LoggedMechanismLigament2d wristLigament;

	public WristSubsystem(WristIO wristIO) {
		this.io = wristIO;
	}

	public void setVoltage(double leadVoltage, double followVoltage) {
		io.setVoltage(leadVoltage, followVoltage);
	}

	public boolean getCoralInput() {
		return io.getCoralInput();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Wrist", inputs);
		getCoralInputBool = getCoralInput();
		SmartDashboard.putBoolean("has coral or summ", getCoralInputBool);
	}
}
