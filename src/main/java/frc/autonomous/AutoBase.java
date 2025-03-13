package frc.autonomous;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
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
import frc.robot.util.Util;
import java.io.File;
import java.util.HashMap;

public abstract class AutoBase extends SequentialCommandGroup {
	public static boolean seenCoral = false;
	public static boolean coralInWrist = false;
	public static boolean coralInPosition = false;
	public static Timer timer = new Timer();
	public static int TotalCommandsAdded = 0;

	// public abstract void init();

	public AutoBase(ElevatorSubsystem elevator, WristSubsystem wrist) {}

	public static PathPlannerPath getPathFromFile(String name) {
		try {
			PathPlannerPath path = PathPlannerPath.fromPathFile(name);
			return path;
		} catch (Exception e) {
			DriverStation.reportError("Cant Find Path : " + e.getMessage(), e.getStackTrace());
			SmartDashboard.putString("PathErrors", "Cant Find Path : " + name);
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
							.03)) {
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
					seenCoral = false;
					coralInWrist = false;
					coralInPosition = false;
					timer.reset();
					wrist.setVoltage(0, 0);
				},
				() -> {
					if (MathUtil.isNear(
							elevator.getTargetPosition(elevator.pos).in(Meters),
							elevator.getPosition().in(Meters),
							.05)) {
						timer.start();
						if (coralInPosition) {
							return;
						}

						if (seenCoral) {
							if (WristSubsystem.getCoralInputBool && !coralInWrist) {
								wrist.setVoltage(WristConstants.slowOuttakeVoltage, WristConstants.slowOuttakeVoltage);
							} else {
								coralInWrist = true;
								if (WristSubsystem.getCoralInputBool) {
									wrist.setVoltage(0, 0);
									coralInPosition = true;
								} else {
									wrist.setVoltage(
											WristConstants.slowIntakeVoltage, WristConstants.slowIntakeVoltage);
								}
							}
						} else {
							if (WristSubsystem.getCoralInputBool) {
								seenCoral = true;
							}
							wrist.setVoltage(WristConstants.intakeVoltage, WristConstants.intakeVoltage);
						}
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
			return AutoBuilder.resetOdom(
					FlippingUtil.flipFieldPose(path.getStartingHolonomicPose().get()));
		}
		return AutoBuilder.resetOdom(path.getStartingHolonomicPose().get());
	}

	public static final Command feedCoralCommand(ElevatorSubsystem elevator, WristSubsystem wrist) {
		setElevatorSetpoint(ElevatorPos.INTAKE, elevator);
		return new ParallelRaceGroup(wristOuttake(wrist, elevator), ElevatorCommands.setPos(elevator));
	}

	public static final class Paths {
		public static HashMap<String, PathPlannerPath> paths = new HashMap<>();

		public static void initPaths() {
			paths.clear();
			File filePath = new File("./src/main/deploy/pathplanner/paths");
			File[] files = filePath.listFiles();
			for (File f : files) {
				SmartDashboard.putString("filePath", f.getName());
				SmartDashboard.putString("fileNoPath", Util.removeFileExtention(f.getName()));
				String s = Util.removeFileExtention(f.getName());
				try {
					PathPlannerPath path = getPathFromFile(s);
					if (path != null) {
						paths.put(s, path);
					} else {
						SmartDashboard.putString("tragic shii 4nem", "that jawn is null");
						SmartDashboard.putString("nullVal at :", s);
					}
				} catch (FileVersionException e) {
					DriverStation.reportError(e.getMessage(), e.getStackTrace());
				}
			}
		}

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
		public static final PathPlannerPath FR_C2 = getPathFromFile("FR to C2");
		public static final PathPlannerPath RM_C2 = getPathFromFile("RM to C2");
		public static final PathPlannerPath R_C2 = getPathFromFile("R to C2");
		public static final PathPlannerPath C2_FeedR = getPathFromFile("C2 to FeedR");
		public static final PathPlannerPath FeedR_C2 = getPathFromFile("FeedR to C2");

		// C3 Paths
		public static final PathPlannerPath FL_C3 = getPathFromFile("FL to C3");
		public static final PathPlannerPath LM_C3 = getPathFromFile("LM to C3");
		public static final PathPlannerPath L_C3 = getPathFromFile("L to C3");
		public static final PathPlannerPath C3_FeedL = getPathFromFile("C3 to FeedL");
		public static final PathPlannerPath FeedL_C3 = getPathFromFile("FeedL to C3");

		// C4 Paths
		public static final PathPlannerPath FL_C4 = getPathFromFile("FL to C4");
		public static final PathPlannerPath LM_C4 = getPathFromFile("LM to C4");
		public static final PathPlannerPath L_C4 = getPathFromFile("L to C4");
		public static final PathPlannerPath C4_FeedL = getPathFromFile("C4 to FeedL");
		public static final PathPlannerPath FeedL_C4 = getPathFromFile("FeedL to C4");

		// C5 Paths
		public static final PathPlannerPath FL_C5 = getPathFromFile("FL to C5");
		public static final PathPlannerPath LM_C5 = getPathFromFile("LM to C5");
		public static final PathPlannerPath L_C5 = getPathFromFile("L to C5");
		public static final PathPlannerPath C5_FeedL = getPathFromFile("C5 to FeedL");
		public static final PathPlannerPath FeedL_C5 = getPathFromFile("FeedL to C5");

		// C6 Paths
		public static final PathPlannerPath FL_C6 = getPathFromFile("FL to C6");
		public static final PathPlannerPath LM_C6 = getPathFromFile("LM to C6");
		public static final PathPlannerPath L_C6 = getPathFromFile("L to C6");
		public static final PathPlannerPath C6_FeedL = getPathFromFile("C6 to FeedL");
		public static final PathPlannerPath FeedL_C6 = getPathFromFile("FeedL to C6");

		// C7 Paths
		public static final PathPlannerPath FR_C7 = getPathFromFile("FR to C7");
		public static final PathPlannerPath RM_C7 = getPathFromFile("RM to C7");
		public static final PathPlannerPath R_C7 = getPathFromFile("R to C7");
		public static final PathPlannerPath FL_C7 = getPathFromFile("FL to C7");
		public static final PathPlannerPath LM_C7 = getPathFromFile("LM to C7");
		public static final PathPlannerPath L_C7 = getPathFromFile("L to C7");
		public static final PathPlannerPath C7_FeedL = getPathFromFile("C7 to FeedL");
		public static final PathPlannerPath FeedL_C7 = getPathFromFile("FeedL to C7");
		public static final PathPlannerPath C7_FeedR = getPathFromFile("C7 to FeedR");
		public static final PathPlannerPath FeedR_C7 = getPathFromFile("FeedR to C7");

		// C8 Paths
		public static final PathPlannerPath FR_C8 = getPathFromFile("FR to C8");
		public static final PathPlannerPath RM_C8 = getPathFromFile("RM to C8");
		public static final PathPlannerPath R_C8 = getPathFromFile("R to C8");
		public static final PathPlannerPath FL_C8 = getPathFromFile("FL to C8");
		public static final PathPlannerPath LM_C8 = getPathFromFile("LM to C8");
		public static final PathPlannerPath L_C8 = getPathFromFile("L to C8");
		public static final PathPlannerPath C8_FeedL = getPathFromFile("C8 to FeedL");
		public static final PathPlannerPath FeedL_C8 = getPathFromFile("FeedL to C8");
		public static final PathPlannerPath C8_FeedR = getPathFromFile("C8 to FeedR");
		public static final PathPlannerPath FeedR_C8 = getPathFromFile("FeedR to C8");

		// C9 Paths
		public static final PathPlannerPath FR_C9 = getPathFromFile("FR to C9");
		public static final PathPlannerPath RM_C9 = getPathFromFile("RM to C9");
		public static final PathPlannerPath R_C9 = getPathFromFile("R to C9");
		public static final PathPlannerPath C9_FeedR = getPathFromFile("C9 to FeedR");
		public static final PathPlannerPath FeedR_C9 = getPathFromFile("FeedR to C9");

		// C10 Paths
		public static final PathPlannerPath FR_C10 = getPathFromFile("FR to C10");
		public static final PathPlannerPath RM_C10 = getPathFromFile("RM to C10");
		public static final PathPlannerPath R_C10 = getPathFromFile("R to C10");
		public static final PathPlannerPath C10_FeedR = getPathFromFile("C10 to FeedR");
		public static final PathPlannerPath FeedR_C10 = getPathFromFile("FeedR to C10");

		// C11 Paths
		public static final PathPlannerPath FR_C11 = getPathFromFile("FR to C11");
		public static final PathPlannerPath RM_C11 = getPathFromFile("RM to C11");
		public static final PathPlannerPath R_C11 = getPathFromFile("R to C11");
		public static final PathPlannerPath C11_FeedR = getPathFromFile("C11 to FeedR");
		public static final PathPlannerPath FeedR_C11 = getPathFromFile("FeedR to C11");

		// C12 Paths
		public static final PathPlannerPath FR_C12 = getPathFromFile("FR to C12");
		public static final PathPlannerPath RM_C12 = getPathFromFile("RM to C12");
		public static final PathPlannerPath R_C12 = getPathFromFile("R to C12");
		public static final PathPlannerPath C12_FeedR = getPathFromFile("C12 to FeedR");
		public static final PathPlannerPath FeedR_C12 = getPathFromFile("FeedR to C12");
	}
}
