package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
	@AutoLog
	public static class AlgaeIOInputs {
		boolean motorConnected = false;
		public AngularVelocity leadVelocity = RPM.of(0);
		public double leadAppliedVolts = 0;
		public double leadCurrentAmps = 0;
	}

	public default void updateInputs(AlgaeIOInputs inputs) {}

	public default void runVoltage(int volts) {}
}
