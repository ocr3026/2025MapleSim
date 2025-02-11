package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.*;

public interface ElevatorIO {
	@AutoLog
	public static class ElevatorIOInputs {
		public boolean masterConnected = false;
		public Distance masterPosition = Inches.of(0);
		public LinearVelocity masterVelocity = InchesPerSecond.of(0);
		public double masterAppliedVolts = 0;
		public double masterCurrentAmps = 0;

		public boolean slaveConnected = false;
		public Distance slavePosition = Inches.of(0);
		public LinearVelocity slaveVelocity = InchesPerSecond.of(0);
		public double slaveAppliedVolts = 0;
		public double slaveCurrentAmps = 0;
	}

	public default void updateInputs(ElevatorIOInputs inputs) {}

	public default void setPosition(Angle position) {}

	public default void setOpenLoop(double output) {}
}
