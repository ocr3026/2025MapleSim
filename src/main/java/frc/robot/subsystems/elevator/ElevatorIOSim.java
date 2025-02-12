package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
	PIDController pid = new PIDController(simkP, 0, simkD);
	private final ElevatorSim elevatorSim = new ElevatorSim(
			simkV, simkA, gearbox, minPosition.in(Meters), maxPosition.in(Meters), true, minPosition.in(Meters));
	double appliedVolts = 0;

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		elevatorSim.update(0.02);

		inputs.masterAppliedVolts = appliedVolts;
		inputs.masterConnected = true;
		inputs.masterCurrentAmps = elevatorSim.getCurrentDrawAmps();
		inputs.masterPosition = Meter.of(elevatorSim.getPositionMeters());
		inputs.masterVelocity = MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
	}

	@Override
	public void setPosition(Distance position) {
		appliedVolts = MathUtil.clamp(pid.calculate(elevatorSim.getPositionMeters(), position.in(Meter)), -12, 12);
		elevatorSim.setInputVoltage(appliedVolts);
	}
}
