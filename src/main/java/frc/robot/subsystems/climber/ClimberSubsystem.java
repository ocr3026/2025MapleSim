package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

	private final ClimberIO io;
	private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

	public ClimberSubsystem(ClimberIO climberIO) {
		this.io = climberIO;
	}

	public void setSpeed(double speed) {
		io.setAngularSpeed(speed);
		SmartDashboard.putNumber("climberSpeed", speed);
	}

	public void runTrapdoor() {
		io.runTrapdoor();
	}

	public void stopTrapdoor() {
		io.stopTrapdoor();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Climber", inputs);
		SmartDashboard.putNumber("Climber position", inputs.winchPosition.in(Degrees));
	}
}
