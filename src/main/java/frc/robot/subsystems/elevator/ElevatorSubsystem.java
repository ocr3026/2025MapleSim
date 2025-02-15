package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorWidth;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class ElevatorSubsystem extends SubsystemBase {
	public int timesRAN = 0;

	public enum ElevatorPos {
		HIGH,
		MID,
		LOW,
		HOME,
		INTAKE;

		// public int wrapHotbar() {
		// 	if(this)
		// }

		public ElevatorPos increment() {
			return switch (this) {
				case HIGH -> HOME;
				case MID -> HIGH;
				case LOW -> MID;
				case INTAKE -> LOW;
				case HOME -> INTAKE;
			};
		}

		public ElevatorPos decrement() {
			return switch (this) {
				case HIGH -> MID;
				case MID -> LOW;
				case LOW -> INTAKE;
				case INTAKE -> HOME;
				case HOME -> HIGH;
			};
		}
	}

	private final ElevatorIO io;
	private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
	public ElevatorPos pos = ElevatorPos.HOME;

	private LoggedMechanism2d mech2d;
	LoggedMechanismRoot2d mechRoot;
	LoggedMechanismLigament2d mechLigament;

	public ElevatorSubsystem(ElevatorIO elevatorIO) {
		this.io = elevatorIO;

		mech2d = new LoggedMechanism2d(elevatorWidth, elevatorHeight);
		mechRoot = mech2d.getRoot("root", 3, 0);
		mechLigament = mechRoot.append(
				new LoggedMechanismLigament2d("elevator", elevatorHeight, 90, 6, new Color8Bit(Color.kBlue)));
	}

	public void setPosition(Distance position) {
		SmartDashboard.putNumber("positionsetpointelevator", (position).in(Meter));
		timesRAN++;
		io.setPosition(position);
	}

	@Override
	public void periodic() {
		// timesRAN++;
		SmartDashboard.putNumber("TIME RAN", timesRAN);
		mechLigament.setLength(inputs.masterPosition.in(Meter));
		SmartDashboard.putNumber("iolength", io.getPosition().in(Meter));
		SmartDashboard.putNumber("elevatorsimlength", mechLigament.getLength());

		io.updateInputs(inputs);
		Logger.processInputs("Elevator", inputs);
		Logger.recordOutput("Elevator/ElevatorMech2d", mech2d);

		io.tick();

		SmartDashboard.putString(
				"CurrentSelectedPos",
				switch (pos) {
					case HIGH -> "HIGH";
					case MID -> "MID";
					case LOW -> "LOW";
					case INTAKE -> "INTAKE";
					case HOME -> "HOME";
					default -> throw new Error("john error");
				});
	}
}
