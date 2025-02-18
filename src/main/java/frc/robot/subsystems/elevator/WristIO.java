package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface WristIO {
	@AutoLog
	public static class WristIOInputs {
		public boolean leftConnected = false;
		public Angle leftPosition = Rotations.of(0.0);
		public AngularVelocity leftVelocity = RPM.of(0);
		public double leftAppliedVolts = 0;
		public double leftCurrentAmps = 0;

		public boolean rightConnected = false;
		public Angle rightPosition = Rotations.of(0.0);
		public AngularVelocity rightVelocity = RPM.of(0);
		public double rightAppliedVolts = 0;
		public double rightCurrentAmps = 0;
	}

	public default void updateInputs(WristIOInputs inputs) {}

	public default void setSpeed(AngularVelocity speed) {}
}
