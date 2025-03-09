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
		public Angle trapdoorPosition = Degrees.of(0);
		public AngularVelocity trapdoorVelocity = DegreesPerSecond.of(0);
		public double trapdoorAppliedVolts = 0;
		public double trapdoorCurrentAmps = 0;
	}

	public default void updateInputs(ClimberIOInputs inputs) {}

	public default void setAngularPosition(Angle position) {}

	public default void setAngularSpeed(double speed) {}

	public default double getPositionInDegrees() {
		return 0;
	}

	public default void runTrapdoor() {}

	public default void stopTrapdoor() {}
}
