// package frc.robot.subsystems.algae;

// import static edu.wpi.first.units.Units.RPM;

// import edu.wpi.first.units.measure.AngularVelocity;
// import org.littletonrobotics.junction.AutoLog;

// public interface AlgaeIO {
// 	@AutoLog
// 	public static class AlgaeIOInputs {
// 		boolean motorConnected = false;
// 		public AngularVelocity motorVelocity = RPM.of(0);
// 		public double motorAppliedVolts = 0;
// 		public double motorCurrentAmps = 0;
// 	}

// 	public default void updateInputs(AlgaeIOInputs inputs) {}

// 	public default void runVoltage(double volts) {}
// }
