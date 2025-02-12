package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorHeight;
import static frc.robot.subsystems.elevator.ElevatorConstants.elevatorWidth;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
	private final ElevatorIO io;
	private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

	private Mechanism2d mech2d;
	MechanismRoot2d mechRoot;
	MechanismLigament2d mechLigament;

	public ElevatorSubsystem(ElevatorIO elevatorIO) {
		this.io = elevatorIO;

		mech2d = new Mechanism2d(elevatorWidth, elevatorHeight);
		mechRoot = mech2d.getRoot("root", 0, 0);
		mechLigament = mechRoot.append(new MechanismLigament2d("elevator", elevatorHeight, 90));
	}

	public void setPosition(Distance position) {
		io.setPosition(position);
	}

	@Override
	public void periodic() {
		io.updateInputs(inputs);
		Logger.processInputs("Elevator", inputs);
		mechLigament.setLength(inputs.masterPosition.in(Meter));
		SmartDashboard.putData("elevatorMech", mech2d);
	}
}
