package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class WristSubsystem extends SubsystemBase {
	private final WristIO io;
	private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

	public static LoggedMechanismLigament2d wristLigament;

	public WristSubsystem(WristIO wristIO) {
		this.io = wristIO;
	}

	public void setVoltage(double leadVoltage, double followVoltage) {
		io.setVoltage(leadVoltage, followVoltage);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Wrist", inputs);
	}
}
