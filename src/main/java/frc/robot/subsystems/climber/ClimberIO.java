import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {
		public boolean connected = false;
		public Angle rotationPosition = Degrees.of(0);
		public AngularVelocity rotationVelocity = DegreesPerSecond.of(0);
		public double appliedVolts = 0;
		public double currentAmps = 0;
	}

	public default void updateInputs(ClimberIOInputs inputs) {}

	public default void setAngularPosition(Angle position) {}
}
