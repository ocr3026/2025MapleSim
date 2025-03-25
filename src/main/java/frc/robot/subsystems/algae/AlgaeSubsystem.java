// package frc.robot.subsystems.algae;

// import static frc.robot.subsystems.algae.AlgaeConstants.kickoutVoltage;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.algae.AlgaeConstants.*;
// import org.littletonrobotics.junction.Logger;

// public class AlgaeSubsystem extends SubsystemBase {
// 	private final AlgaeIO io;
// 	private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

// 	public AlgaeSubsystem(AlgaeIO io) {
// 		this.io = io;
// 	}

// 	public Command runAlgaeManipulator() {
// 		return Commands.runEnd(
// 				() -> {
// 					io.runVoltage(kickoutVoltage);
// 				},
// 				() -> {
// 					io.runVoltage(0);
// 				},
// 				this);
// 	}

// 	@Override
// 	public void periodic() {
// 		io.updateInputs(inputs);
// 		Logger.processInputs("Algae", inputs);
// 	}
// }
