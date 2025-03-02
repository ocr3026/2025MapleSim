package frc.autonomous;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public abstract class AutoBase {
	public static Timer timer = new Timer();
	public static SequentialCommandGroup group = new SequentialCommandGroup();

	public abstract void init();

	public static PathPlannerPath getPathFromFile(String name) {
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile(name);
			return path;
		} catch (Exception e) {
			DriverStation.reportError("Cant Find Path : " + e.getMessage(), e.getStackTrace());
			return null;
		}
	}

	public static final class Paths {
		public static final PathPlannerPath TEST_PATH = getPathFromFile("Test 1");
		public static final PathPlannerPath TEST_PATH_2 = getPathFromFile("Test 2");
	}

	public static final Command wait(double time) {
		return new WaitCommand(time);
	}

	public static final Command delayStartTime() {
		return new FunctionalCommand(
				() -> {
					timer.reset();
					timer.start();
				},
				() -> {},
				(interupted) -> {
					timer.stop();
					timer.reset();
				},
				() -> {
					return timer.hasElapsed(SmartDashboard.getNumber("delayStartTime", 0));
				});
	}

	public static final Command wristOuttake(WristSubsystem wrist, ElevatorSubsystem elevator) {
		return new FunctionalCommand(
				() -> {
					timer.reset();
					wrist.setVoltage(0);
				},
				() -> {
					if (MathUtil.isNear(
							elevator.getTargetPosition(elevator.pos).in(Meters),
							elevator.getPosition().in(Meters),
							.05)) {
						timer.start();
						wrist.setVoltage(WristConstants.outtakeVoltage);
					}
				},
				(interupted) -> {
					wrist.setVoltage(0);
					timer.stop();
					timer.reset();
				},
				() -> {
					return timer.hasElapsed(1.5);
				});
	}

	public static final Command wristIntake(WristSubsystem wrist, ElevatorSubsystem elevator) {
		return new FunctionalCommand(
				() -> {
					timer.reset();
					wrist.setVoltage(0);
				},
				() -> {
					if (MathUtil.isNear(
							elevator.getTargetPosition(elevator.pos).in(Meters),
							elevator.getPosition().in(Meters),
							.05)) {
						timer.start();
						wrist.setVoltage(WristConstants.intakeVoltage);
					}
				},
				(interupted) -> {
					wrist.setVoltage(0);
					timer.stop();
					timer.reset();
				},
				() -> {
					return timer.hasElapsed(1.5);
				});
	}

	public static final Command setElevatorPosition(ElevatorSubsystem subsystem, ElevatorPos pos) {
		return new FunctionalCommand(
				() -> {},
				() -> {
					subsystem.pos = pos;
				},
				(interupted) -> {},
				() -> {
					return subsystem.pos == pos;
				});
	}

	public static final ParallelRaceGroup moveElevatorAndOuttake(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		return new ParallelRaceGroup(wristOuttake(wrist, elevator), ElevatorCommands.setPos(elevator));
	}

	public static final ParallelRaceGroup moveElevatorAndIntake(WristSubsystem wrist, ElevatorSubsystem elevator) {
		return new ParallelRaceGroup(wristIntake(wrist, elevator), ElevatorCommands.setPos(elevator));
	}

	public static final Command setStartPose(PathPlannerPath path) {
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return AutoBuilder.resetOdom(FlippingUtil.flipFieldPose(path.getStartingDifferentialPose()));
		}
		return AutoBuilder.resetOdom(path.getStartingDifferentialPose());
	}
}
