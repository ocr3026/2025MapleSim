package frc.robot.subsystems.climber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

	private final ClimberIO io;
	private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

	public ClimberSubsystem(ClimberIO climberIO) {
		this.io = climberIO;
	}

	public void setPosition(Angle position) {
		io.setAngularPosition(position);
	}

	public void setSpeed(double speed) {
		io.setAngularSpeed(speed);
	}

	public double getPositionInDegrees() {
		return io.getPositionInDegrees();
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Climber", inputs);
		SmartDashboard.putNumber("ClimberPos", getPositionInDegrees());
	}
}
