package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
	@AutoLog
	public static class WristIOInputs {
		public boolean leadConnected = false;
		public Angle leadPosition = Rotations.of(0.0);
		public AngularVelocity leadVelocity = RPM.of(0);
		public double leadAppliedVolts = 0;
		public double leadCurrentAmps = 0;

		public boolean followConnected = false;
		public Angle followPosition = Rotations.of(0.0);
		public AngularVelocity followVelocity = RPM.of(0);
		public double followAppliedVolts = 0;
		public double followCurrentAmps = 0;
	}

	public default void updateInputs(WristIOInputs inputs) {}

	public default void setVoltage(double leadVoltage, double followVoltage) {}

	public default boolean getCoralInput() {
		return false;
	}
}
