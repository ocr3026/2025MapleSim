package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorWidth;
import static frc.robot.subsystems.elevator.ElevatorConstants.softwareLimit;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.wrist.WristSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ElevatorSubsystem extends SubsystemBase {
	public int timesRAN = 0;

	public enum ElevatorPos {
		HIGH,
		MIDALGAE,
		MID,
		LOWALGAE,
		LOW,
		INTAKE;

		public ElevatorPos increment() {
			return switch (this) {
				case HIGH -> INTAKE;
				case MIDALGAE -> LOW;
				case MID -> HIGH;
				case LOWALGAE -> MIDALGAE;
				case LOW -> MID;
				case INTAKE -> LOW;
			};
		}

		public ElevatorPos decrement() {
			return switch (this) {
				case HIGH -> MID;
				case MIDALGAE -> LOWALGAE;
				case MID -> LOW;
				case LOWALGAE -> INTAKE;
				case LOW -> INTAKE;
				case INTAKE -> HIGH;
			};
		}
	}

	public static final LoggedDashboardChooser<ElevatorPos> firstElevatorPosChooser =
			new LoggedDashboardChooser<>("Choose First Elevator Pos");
	public static final LoggedDashboardChooser<ElevatorPos> secondElevatorPosChooser =
			new LoggedDashboardChooser<>("Choose Second Elevator Pos");
	public static final LoggedDashboardChooser<ElevatorPos> thirdElevatorPosChooser =
			new LoggedDashboardChooser<>("Choose Third Elevator Pos");
	public static final LoggedDashboardChooser<ElevatorPos> fourthElevatorPosChooser =
			new LoggedDashboardChooser<>("Choose Fourth Elevator Pos");

	private final ElevatorIO io;
	private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
	public ElevatorPos pos = ElevatorPos.INTAKE;

	private LoggedMechanism2d mech2d;
	public static LoggedMechanismRoot2d mechRoot;
	public static LoggedMechanismLigament2d mechLigament;

	public ElevatorSubsystem(ElevatorIO elevatorIO) {
		this.io = elevatorIO;

		mech2d = new LoggedMechanism2d(elevatorWidth.in(Meters), elevatorHeight.in(Meters));
		mechRoot = mech2d.getRoot("root", 0, 0);
		mechLigament = mechRoot.append(new LoggedMechanismLigament2d(
				"elevator", elevatorHeight.in(Inches), 90, 6, new Color8Bit(Color.kBlue)));

		WristSubsystem.wristLigament = mechLigament.append(
				new LoggedMechanismLigament2d("Wrist", 0.3, 210, 6, new Color8Bit(Color.kDarkGreen)));
	}

	public void setPosition(Distance position) {
		try {
			SmartDashboard.putNumber("positionsetpointelevator", (position).in(Meter));
			timesRAN++;
			io.setPosition(position);
		} catch (NullPointerException e) {
			DriverStation.reportError("Position is null", e.getStackTrace());
		}
	}

	public void setSpeed(double speed) {
		io.setSpeed(speed);
	}

	public Distance getPosition() {
		return io.getPosition();
	}

	public Distance getTargetPosition(ElevatorPos givenPos) {
		return io.getTargetPosition(givenPos);
	}

	public static final void initElevatorEnum() {
		for (ElevatorPos pos : ElevatorPos.values()) {
			pos.name();
			firstElevatorPosChooser.addOption(pos.name(), pos);
			secondElevatorPosChooser.addOption(pos.name(), pos);
			thirdElevatorPosChooser.addOption(pos.name(), pos);
			fourthElevatorPosChooser.addOption(pos.name(), pos);
		}
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("midAlgaePOS", ElevatorCommands.midAlgaePOS.in(Meters));
		SmartDashboard.putNumber("lowAlgaePOS", ElevatorCommands.lowAlgaePOS.in(Meters));
		SmartDashboard.putNumber("midAlgaeConstPOS", ElevatorConstants.midAlgaePosConst.in(Meters));
		SmartDashboard.putNumber("lowAlgaeConstPOS", ElevatorConstants.lowAlgaePosConst.in(Meters));
		// timesRAN++;
		SmartDashboard.putNumber("TIME RAN", timesRAN);
		mechLigament.setLength(inputs.masterPosition.in(Meter));
		SmartDashboard.putNumber("iolength", io.getPosition().in(Meters));
		SmartDashboard.putNumber("elevatorsimlength", mechLigament.getLength());
		SmartDashboard.putNumber("Max Height", softwareLimit.in(Meters));
		SmartDashboard.putNumber("SetpointNum", io.getTargetPosition(pos).in(Meters));

		io.updateInputs(inputs);
		Logger.processInputs("Elevator", inputs);
		Logger.recordOutput("Elevator/ElevatorMech2d", mech2d);

		if (Constants.currentMode != Constants.Mode.SIM) {
			if (io.getPosition().in(Meters) <= softwareLimit.in(Meters)) {
				io.tick();
			} else {
				io.tick();
			}
		} else {
			io.tick();
		}

		SmartDashboard.putString(
				"CurrentSelectedPos",
				switch (pos) {
					case HIGH -> "HIGH";
					case MIDALGAE -> "MIDALGAE";
					case MID -> "MID";
					case LOWALGAE -> "LOWALGAE";
					case LOW -> "LOW";
					case INTAKE -> "INTAKE";
					default -> throw new Error("john error");
				});

		SmartDashboard.putString("Elevator.pos", pos.toString());
	}
}
