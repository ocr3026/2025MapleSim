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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;

public abstract class AutoBase extends SequentialCommandGroup {
	public static Timer timer = new Timer();
	public static int TotalCommandsAdded = 0;

	// public abstract void init();

	public static PathPlannerPath getPathFromFile(String name) {
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile(name);
			return path;
		} catch (Exception e) {
			DriverStation.reportError("Cant Find Path : " + e.getMessage(), e.getStackTrace());
			return null;
		}
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
					wrist.setVoltage(0, 0);
				},
				() -> {
					if (MathUtil.isNear(
							elevator.getTargetPosition(elevator.pos).in(Meters),
							elevator.getPosition().in(Meters),
							.05)) {
						timer.start();
						wrist.setVoltage(WristConstants.outtakeVoltage, WristConstants.outtakeVoltage);
					}
				},
				(interupted) -> {
					wrist.setVoltage(0, 0);
					timer.stop();
					timer.reset();
				},
				() -> {
					return timer.hasElapsed(1.5);
				});
	}

	public static final Command wristOuttakeHome(WristSubsystem wrist, ElevatorSubsystem elevator) {
		return new FunctionalCommand(
				() -> {
					timer.reset();
					wrist.setVoltage(0, 0);
				},
				() -> {
					if (MathUtil.isNear(
							elevator.getTargetPosition(elevator.pos).in(Meters),
							elevator.getPosition().in(Meters),
							.05)) {
						timer.start();
						wrist.setVoltage(WristConstants.outtakeVoltage, 0.1);
					}
				},
				(interupted) -> {
					wrist.setVoltage(0, 0);
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
					wrist.setVoltage(0, 0);
				},
				() -> {
					if (MathUtil.isNear(
							elevator.getTargetPosition(elevator.pos).in(Meters),
							elevator.getPosition().in(Meters),
							.05)) {
						timer.start();
						wrist.setVoltage(WristConstants.intakeVoltage, WristConstants.intakeVoltage);
					}
				},
				(interupted) -> {
					wrist.setVoltage(0, 0);
					timer.stop();
					timer.reset();
				},
				() -> {
					return timer.hasElapsed(1.5);
				});
	}

	public static final Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command setElevatorSetpoint(ElevatorPos setPos, ElevatorSubsystem elevator) {
		return Commands.runOnce(() -> elevator.pos = setPos);
	}

	public static final ParallelCommandGroup moveElevatorAndOuttake(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		elevator.pos = pos;
		SmartDashboard.putString("Elevator.pos", elevator.pos.toString());

		return new ParallelCommandGroup(
				setElevatorSetpoint(pos, elevator),
				new ParallelRaceGroup(wristOuttake(wrist, elevator), ElevatorCommands.setPos(elevator)));
	}

	public static final ParallelCommandGroup moveElevatorAndOuttakeHome(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		elevator.pos = pos;
		SmartDashboard.putString("Elevator.pos", elevator.pos.toString());

		return new ParallelCommandGroup(
				setElevatorSetpoint(pos, elevator),
				new ParallelRaceGroup(wristOuttakeHome(wrist, elevator), ElevatorCommands.setPos(elevator)));
	}

	public static final ParallelRaceGroup moveElevatorAndIntake(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		elevator.pos = pos;
		return new ParallelRaceGroup(wristIntake(wrist, elevator), ElevatorCommands.setPos(elevator));
	}

	public static final Command setStartPose(PathPlannerPath path) {
		if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
			return AutoBuilder.resetOdom(FlippingUtil.flipFieldPose(path.getStartingDifferentialPose()));
		}
		return AutoBuilder.resetOdom(path.getStartingDifferentialPose());
	}

	public static final Command feedCoralCommand(ElevatorSubsystem elevator, WristSubsystem wrist) {
		setElevatorSetpoint(ElevatorPos.INTAKE, elevator);
		return new ParallelRaceGroup(wristOuttake(wrist, elevator), ElevatorCommands.setPos(elevator));
	}

	public static final class Paths {
		public static final PathPlannerPath TEST_PATH = getPathFromFile("Test 1");
		public static final PathPlannerPath TEST_PATH_2 = getPathFromFile("Test 2");
		public static final PathPlannerPath FEED_TO_CC = getPathFromFile("Feed to Coral Close Path");
		public static final PathPlannerPath CC_TO_FEED = getPathFromFile("Coral to Feed Path");
		public static final PathPlannerPath CORAL_FL_PATH = getPathFromFile("Coral Far Left Path");

		// C1 Paths
		public static final PathPlannerPath FR_C1 = getPathFromFile("FR to C1");
		public static final PathPlannerPath RM_C1 = getPathFromFile("RM to C1");
		public static final PathPlannerPath R_C1 = getPathFromFile("R to C1");
		public static final PathPlannerPath FL_C1 = getPathFromFile("FL to C1");
		public static final PathPlannerPath LM_C1 = getPathFromFile("LM to C1");
		public static final PathPlannerPath L_C1 = getPathFromFile("L to C1");
		public static final PathPlannerPath C1_FeedL = getPathFromFile("C1 to FeedL");
		public static final PathPlannerPath FeedL_C1 = getPathFromFile("FeedL to C1");
		public static final PathPlannerPath C1_FeedR = getPathFromFile("C1 to FeedR");
		public static final PathPlannerPath FeedR_C1 = getPathFromFile("FeedR to C1");

		// C2 Paths
		public static final PathPlannerPath FL_C2 = getPathFromFile("FL to C2");
		public static final PathPlannerPath LM_C2 = getPathFromFile("LM to C2");
		public static final PathPlannerPath L_C2 = getPathFromFile("L to C2");
		public static final PathPlannerPath C2_FeedL = getPathFromFile("C2 to FeedL");
		public static final PathPlannerPath FeedL_C2 = getPathFromFile("FeedL to C2");

		// C3 Paths
		public static final PathPlannerPath FL_C3 = getPathFromFile("FL to C3");
		public static final PathPlannerPath LM_C3 = getPathFromFile("LM to C3");
		public static final PathPlannerPath L_C3 = getPathFromFile("L to C3");
		public static final PathPlannerPath C3_FeedL = getPathFromFile("C3 to FeedL");
		public static final PathPlannerPath FeedL_C3 = getPathFromFile("FeedL to C3");

		// C4 Paths
		public static final PathPlannerPath FR_C4 = getPathFromFile("FR to C4");
		public static final PathPlannerPath RM_C4 = getPathFromFile("RM to C4");
		public static final PathPlannerPath R_C4 = getPathFromFile("R to C4");
		public static final PathPlannerPath FL_C4 = getPathFromFile("FL to C4");
		public static final PathPlannerPath LM_C4 = getPathFromFile("LM to C4");
		public static final PathPlannerPath L_C4 = getPathFromFile("L to C4");
		public static final PathPlannerPath C4_FeedL = getPathFromFile("C4 to FeedL");
		public static final PathPlannerPath FeedL_C4 = getPathFromFile("FeedL to C4");
		public static final PathPlannerPath C4_FeedR = getPathFromFile("C4 to FeedR");
		public static final PathPlannerPath FeedR_C4 = getPathFromFile("FeedR to C4");

		// C5 Paths
		public static final PathPlannerPath FR_C5 = getPathFromFile("FR to C5");
		public static final PathPlannerPath RM_C5 = getPathFromFile("RM to C5");
		public static final PathPlannerPath R_C5 = getPathFromFile("R to C5");
		public static final PathPlannerPath C5_FeedR = getPathFromFile("C5 to FeedR");
		public static final PathPlannerPath FeedR_C5 = getPathFromFile("FeedR to C5");

		// C6 Paths
		public static final PathPlannerPath FR_C6 = getPathFromFile("FR to C6");
		public static final PathPlannerPath RM_C6 = getPathFromFile("RM to C6");
		public static final PathPlannerPath R_C6 = getPathFromFile("R to C6");
		public static final PathPlannerPath C6_FeedR = getPathFromFile("C6 to FeedR");
		public static final PathPlannerPath FeedR_C6 = getPathFromFile("FeedR to C6");
	}
}
