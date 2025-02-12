package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
	@AutoLog
	public static class ElevatorIOInputs {
		public boolean masterConnected = false;
		public Distance masterPosition = Inches.of(0);
		public LinearVelocity masterVelocity = InchesPerSecond.of(0);
		public double masterAppliedVolts = 0;
		public double masterCurrentAmps = 0;
	}

	public default void updateInputs(ElevatorIOInputs inputs) {}

	public default void setPosition(Distance position) {}
}
