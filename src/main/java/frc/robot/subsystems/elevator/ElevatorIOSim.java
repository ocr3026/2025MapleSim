package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;

public class ElevatorIOSim implements ElevatorIO {
	PIDController pid = new PIDController(simkP, 0, simkD);
	private final ElevatorSim elevatorSim = new ElevatorSim(
			simkV, simkA, gearbox, minPosition.in(Meter), maxPosition.in(Meter), false, minPosition.in(Meter));
	double appliedVolts = 0;
	public Distance setpoint = Meter.of(0);

	@Override
	public void updateInputs(ElevatorIOInputs inputs) {
		elevatorSim.update(0.02);

		inputs.masterAppliedVolts = appliedVolts;
		inputs.masterConnected = true;
		inputs.masterCurrentAmps = elevatorSim.getCurrentDrawAmps();
		inputs.masterPosition = Meter.of(elevatorSim.getPositionMeters());
		inputs.masterVelocity = MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());

		inputs.followAppliedVolts = appliedVolts;
		inputs.followConnected = true;
		inputs.followCurrentAmps = elevatorSim.getCurrentDrawAmps();
		inputs.followPosition = Meter.of(elevatorSim.getPositionMeters());
		inputs.followVelocity = MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
	}

	@Override
	public void setPosition(Distance position) {
		setpoint = position;
		// SmartDashboard.putNumber("PIDOUTPUT", pid.calculate(elevatorSim.getPositionMeters(), position.in(Meter)));
		// appliedVolts = MathUtil.clamp(pid.calculate(elevatorSim.getPositionMeters(), position.in(Meter)), -12, 12);
		// SmartDashboard.putNumber("elevatorSIM", currentLimit);
		// elevatorSim.setInputVoltage(appliedVolts);
		// elevatorSim.setInput(MathUtil.clamp(pid.calculate(elevatorSim.getPositionMeters(), position.in(Meter)), -1,
		// 1));
	}

	@Override
	public Distance getPosition() {
		return Meters.of((elevatorSim.getPositionMeters()));
	}

	@Override
	public void tick() {
		// if (!(setpoint.in(Meter) - 0.01 <= elevatorSim.getPositionMeters())
		//		&& !(elevatorSim.getPositionMeters() <= setpoint.in(Meter) + 0.01)) {
		SmartDashboard.putNumber("PIDOUTPUT", pid.calculate(elevatorSim.getPositionMeters(), setpoint.in(Meter)));
		SmartDashboard.putNumber("PIDCALCSIMELEVATOR", appliedVolts);
		appliedVolts = MathUtil.clamp(
				pid.calculate(elevatorSim.getPositionMeters(), setpoint.in(Meter) + minPosition.in(Meters)), -12, 12);
		elevatorSim.setInputVoltage(appliedVolts);
		// }
	}

	@Override
	public Distance getTargetPosition(ElevatorPos givenPos) {
		return (setpoint.plus(minPosition));
	}

	@Override
	public void setSpeed(double speed) {}
}
