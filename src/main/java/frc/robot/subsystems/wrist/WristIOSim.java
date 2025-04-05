package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.wrist.WristConstants.*;

import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class WristIOSim implements WristIO {
	Angle leadPosition = Rotations.of(0);
	private final FlywheelSim flywheelSim =
			new FlywheelSim(LinearSystemId.createFlywheelSystem(gearbox, 0.00015, (1.0 / 10.0)), gearbox);

	/** @param inputs */
	@Override
	public void updateInputs(WristIOInputs inputs) {
		flywheelSim.update(0.02);

		inputs.leadConnected = true;
		inputs.leadAppliedVolts = flywheelSim.getInputVoltage();
		inputs.leadCurrentAmps = flywheelSim.getCurrentDrawAmps();
		leadPosition = (inputs.leadPosition.plus(Rotations.of(flywheelSim.getAngularVelocityRPM() * (0.02 / 60))));
		inputs.leadPosition = inputs.leadPosition.plus(Rotations.of(flywheelSim.getAngularVelocityRPM() * (0.02 / 60)));
		inputs.leadVelocity = flywheelSim.getAngularVelocity();

		inputs.followConnected = true;
		inputs.followAppliedVolts = flywheelSim.getInputVoltage();
		inputs.followCurrentAmps = flywheelSim.getCurrentDrawAmps();
		inputs.followPosition =
				inputs.followPosition.plus(Rotations.of(flywheelSim.getAngularVelocityRPM() * (0.02 / 60)));
		inputs.followVelocity = flywheelSim.getAngularVelocity();
	}

	/**
	 * @param leadVoltage
	 * @param followVoltage
	 */
	@Override
	public void setVoltage(double leadVoltage, double followVoltage) {
		flywheelSim.setInputVoltage(leadVoltage);
	}

	/** @return boolean */
	@Override
	public boolean getCoralInput() {
		return true;
	}

	/** @return double */
	@Override
	public double getRotations() {
		return leadPosition
				.plus(Rotations.of(flywheelSim.getAngularVelocityRPM() * (0.02 / 60)))
				.in(Rotations);
	}
}
