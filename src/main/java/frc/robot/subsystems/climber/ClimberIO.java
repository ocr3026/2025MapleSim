package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {
		public boolean winchConnected = false;
		public Angle winchPosition = Degrees.of(0);
		public AngularVelocity winchVelocity = DegreesPerSecond.of(0);
		public double winchAppliedVolts = 0;
		public double winchCurrentAmps = 0;

		public boolean trapdoorConnected = false;
		public double trapdoorPosition = 0;
		public AngularVelocity trapdoorVelocity = RPM.of(0);
		public double trapdoorAppliedVolts = 0;
		public double trapdoorCurrentAmps = 0;
	}

	public default void updateInputs(ClimberIOInputs inputs) {}

	public default void setAngularSpeed(double speed) {}

	public default void runTrapdoor(double voltage) {}

	public default void stopTrapdoor() {}
}
