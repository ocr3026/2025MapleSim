package frc.autonomous;

import static edu.wpi.first.units.Units.Meters;
import static frc.autonomous.AutoBase.Paths.pathsHaveInit;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.autonomous.AutoBase.Paths;
import frc.robot.Constants;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorPos;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.util.Util;
import java.io.File;
import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public abstract class AutoBase extends SequentialCommandGroup {
	public static boolean seenCoral = false;
	public static boolean coralInWrist = false;
	public static boolean coralInPosition = false;
	public static Timer timer = new Timer();
	public static int TotalCommandsAdded = 0;

	// public abstract void init();

	public AutoBase(ElevatorSubsystem elevator, WristSubsystem wrist) {}

	
	/** 
	 * @param name
	 * @return PathPlannerPath
	 */
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

	
	/** 
	 * @param time
	 * @return Command
	 */
	public static final Command wait(double time) {
		return new WaitCommand(time);
	}

	
	/** 
	 * @return Command
	 */
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

	
	/** 
	 * @param elevator
	 * @param pos
	 * @return SequentialCommandGroup
	 */
	public static final SequentialCommandGroup moveElevator(ElevatorSubsystem elevator, ElevatorPos pos) {
		return new SequentialCommandGroup(setElevatorSetpoint(pos, elevator), ElevatorCommands.setPos(elevator));
	}
	
	
	/** 
	 * @param elevator
	 * @param pos
	 * @param path
	 * @return ParallelDeadlineGroup
	 */
	public static final ParallelDeadlineGroup followPathandMoveElevator(ElevatorSubsystem elevator, ElevatorPos pos, PathPlannerPath path) {
		return new ParallelDeadlineGroup(
			followPath(path), moveElevator(elevator, pos));
	}

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @return Command
	 */
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
							.01)) {
						timer.start();
						if (timer.hasElapsed(0.3)) {
							wrist.setVoltage(WristConstants.outtakeVoltage, WristConstants.outtakeVoltage);
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

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @return Command
	 */
	public static final Command wristOuttakeHomeRight(WristSubsystem wrist, ElevatorSubsystem elevator) {
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
						wrist.setVoltage(-3, -0.5);
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

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @return Command
	 */
	public static final Command wristOuttakeHomeLeft(WristSubsystem wrist, ElevatorSubsystem elevator) {
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
						wrist.setVoltage(-0.3, -3);
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

	public static boolean timedOut = false;

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @return Command
	 */
	public static final Command wristIntake(WristSubsystem wrist, ElevatorSubsystem elevator) {
		return new FunctionalCommand(
				// INIT
				() -> {
					seenCoral = false;
					coralInWrist = false;
					coralInPosition = false;
					timedOut = false;
					timer.reset();
					SmartDashboard.putBoolean("Has timed out", timedOut);

					wrist.setVoltage(0, 0);
				},
				// EXECUTE
				() -> {
					if (MathUtil.isNear(
							elevator.getTargetPosition(elevator.pos).in(Meters),
							elevator.getPosition().in(Meters),
							.05)) {
						timer.start();

						if (seenCoral || timer.hasElapsed(1.0)) {
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
				// END
				(interupted) -> {
					if (timer.hasElapsed(1.5)) {
						timedOut = true;
					}
					wrist.setVoltage(0, 0);
					timer.stop();
					timer.reset();
					SmartDashboard.putBoolean("Has timed out", timedOut);
				},

				// INTERUPTED

				() -> {
					return timer.hasElapsed(1.5) || coralInPosition;
				});
	}

	
	/** 
	 * @param path
	 * @return Command
	 */
	public static final Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	
	/** 
	 * @param setPos
	 * @param elevator
	 * @return Command
	 */
	public static Command setElevatorSetpoint(ElevatorPos setPos, ElevatorSubsystem elevator) {
		return Commands.runOnce(() -> elevator.pos = setPos);
	}

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @param pos
	 * @return ParallelCommandGroup
	 */
	public static final ParallelCommandGroup moveElevatorAndOuttake(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		elevator.pos = pos;
		SmartDashboard.putString("Elevator.pos", elevator.pos.toString());
		// if (pos == ElevatorPos.INTAKE) {
		// 	return new ParallelCommandGroup(
		// 			Commands.runOnce(() -> SmartDashboard.putString("outtakeMode", "outtake trough")),
		// 			setElevatorSetpoint(pos, elevator),
		// 			new ParallelRaceGroup(wristOuttakeHomeRight(wrist, elevator), ElevatorCommands.setPos(elevator)));
		// } else {
		return new ParallelCommandGroup(
				Commands.runOnce(() -> SmartDashboard.putString("outtakeMode", "outtake coral")),
				setElevatorSetpoint(pos, elevator),
				new ParallelRaceGroup(wristOuttake(wrist, elevator), ElevatorCommands.setPos(elevator)));
		// }
	}

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @param pos
	 * @return ParallelCommandGroup
	 */
	public static final ParallelCommandGroup moveElevatorAndOuttakeHomeRight(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		elevator.pos = pos;
		SmartDashboard.putString("Elevator.pos", elevator.pos.toString());

		return new ParallelCommandGroup(
				setElevatorSetpoint(pos, elevator),
				new ParallelRaceGroup(wristOuttakeHomeRight(wrist, elevator), ElevatorCommands.setPos(elevator)));
	}

	
	/** 
	 * @param wrist WristSubsystem
	 * @param elevator ElevatorSubsystem
	 * @param pos ElevatorPos
	 * @return ParallelCommandGroup to move the elevator and outtake in trough with coral going to the left
	 */
	public static final ParallelCommandGroup moveElevatorAndOuttakeHomeLeft(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		elevator.pos = pos;
		SmartDashboard.putString("Elevator.pos", elevator.pos.toString());

		return new ParallelCommandGroup(
				setElevatorSetpoint(pos, elevator),
				new ParallelRaceGroup(wristOuttakeHomeLeft(wrist, elevator), ElevatorCommands.setPos(elevator)));
	}

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @param pos
	 * @return ParallelRaceGroup
	 */
	public static final ParallelRaceGroup moveElevatorAndIntake(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		elevator.pos = pos;
		return new ParallelRaceGroup(wristIntake(wrist, elevator), ElevatorCommands.setPos(elevator));
	}

	
	/** 
	 * @param wrist
	 * @param elevator
	 * @param pos
	 * @return ParallelCommandGroup
	 */
	public static final ParallelCommandGroup moveElevatorAndIntakeNoRace(
			WristSubsystem wrist, ElevatorSubsystem elevator, ElevatorPos pos) {
		return new ParallelCommandGroup(
				setElevatorSetpoint(pos, elevator), wristIntake(wrist, elevator), ElevatorCommands.setPos(elevator));
	}

	
	/** 
	 * @param path
	 * @return Command
	 */
	public static final Command setStartPose(PathPlannerPath path) {
		Pose2d holoPose = path.getStartingHolonomicPose().get();

		// if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
		// 	return AutoBuilder.resetOdom(FlippingUtil.flipFieldPose(holoPose));
		// }
		// if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
		// 	return AutoBuilder.resetOdom(FlippingUtil.flipFieldPose(holoPose));
		// }
		return AutoBuilder.resetOdom(holoPose);
	}

	
	/** 
	 * @param elevator
	 * @param wrist
	 * @return Command
	 */
	public static final Command feedCoralCommand(ElevatorSubsystem elevator, WristSubsystem wrist) {
		return new ParallelCommandGroup(
				setElevatorSetpoint(ElevatorPos.INTAKE, elevator),
				moveElevatorAndIntake(wrist, elevator, ElevatorPos.INTAKE));
	}

	
	/** 
	 * @param path
	 * @return PathPlannerPath
	 */
	public static final PathPlannerPath getPathToFeed(PathPlannerPath path) {
		String name = path.name;
		int index = name.indexOf("C");
		String subString;
		if (name.contains("12") || name.contains("11") || name.contains("10")) {
			subString = name.substring(index, (index + 3));
		} else {
			subString = name.substring(index, (index + 2));
		}
		SmartDashboard.putString("fixedPath", subString);
		for (PathPlannerPath p : Paths.paths.values()) {
			if (p.name.contains(subString) && p.name.contains("Feed") && p.name.indexOf("F") > 3) {
				SmartDashboard.putString("fixPath", p.name);

				return p;
			}
		}
		return Paths.paths.get("Do Nothing");
	}

	public static final class Paths {
		// public static List<Pose2d> coralPosesRight = new ArrayList<Pose2d>();
		// public static List<Pose2d> coralPosesLeft = new ArrayList<Pose2d>();
		public static HashMap<Pose2d, String> coralPosesRight = new HashMap<>();
		public static HashMap<Pose2d, String> coralPosesLeft = new HashMap<>();

		public static boolean pathsHaveInit = false;

		public static HashMap<String, PathPlannerPath> paths = new HashMap<>();
		public static PathPlannerPath pathsArray[];

		public static void initPaths() {
			File filePath;
			paths.clear();
			if (Constants.currentMode == Constants.Mode.SIM) {
				filePath = new File("./src/main/deploy/pathplanner/paths");
			} else {
				filePath = new File("/home/lvuser/deploy/pathplanner/paths");
			}
			File[] files = filePath.listFiles();
			for (File f : files) {
				SmartDashboard.putString("filePath", f.getName());
				SmartDashboard.putString("fileNoPath", Util.removeFileExtention(f.getName()));
				String s = Util.removeFileExtention(f.getName());
				try {
					PathPlannerPath path = getPathFromFile(s);
					if (path != null) {
						paths.put(s, path);
					}
				} catch (FileVersionException e) {
					DriverStation.reportError(e.getMessage(), e.getStackTrace());
				}
			}
		}
		/*
		 * Choose paths based on what coral spots you want to go to, auto get the path to the feed and then the next path will be from the feed to the coral
		 * Ex: First Path chooses starting pos AND coral spot
		 * Second Path only chooses coral, same with third.
		 */
		public static final LoggedDashboardChooser<PathPlannerPath> firstPathChooser =
				new LoggedDashboardChooser<>("Choose First Path");
		public static final LoggedDashboardChooser<PathPlannerPath> secondPathChooser =
				new LoggedDashboardChooser<>("Choose Second Path");
		public static final LoggedDashboardChooser<PathPlannerPath> thirdPathChooser =
				new LoggedDashboardChooser<>("Choose Third Path");

		public static final LoggedDashboardChooser<ElevatorPos> firstElevatorPosChooser =
				new LoggedDashboardChooser<>("Choose First Elevator Pos");
		public static final LoggedDashboardChooser<ElevatorPos> secondElevatorPosChooser =
				new LoggedDashboardChooser<>("Choose Second Elevator Pos");
		public static final LoggedDashboardChooser<ElevatorPos> thirdElevatorPosChooser =
				new LoggedDashboardChooser<>("Choose Third Elevator Pos");
		public static final LoggedDashboardChooser<ElevatorPos> fourthElevatorPosChooser =
				new LoggedDashboardChooser<>("Choose Fourth Elevator Pos");

		public static final void initElevatorEnum() {
			for (ElevatorPos pos : ElevatorPos.values()) {
				pos.name();
				firstElevatorPosChooser.addOption(pos.name(), pos);
				secondElevatorPosChooser.addOption(pos.name(), pos);
				thirdElevatorPosChooser.addOption(pos.name(), pos);
				fourthElevatorPosChooser.addOption(pos.name(), pos);
			}
		}

		public static void initAutoFactory() {
			firstPathChooser.addDefaultOption("Default", paths.get("Drive forward slow"));
			secondPathChooser.addDefaultOption("Default", paths.get("Drive forward slow"));
			thirdPathChooser.addDefaultOption("Default", paths.get("Drive forward slow"));
			firstElevatorPosChooser.addDefaultOption("Default", ElevatorPos.INTAKE);
			secondElevatorPosChooser.addDefaultOption("Default", ElevatorPos.INTAKE);
			thirdElevatorPosChooser.addDefaultOption("Default", ElevatorPos.INTAKE);
			fourthElevatorPosChooser.addDefaultOption("Default", ElevatorPos.INTAKE);

			initElevatorEnum();

			for (PathPlannerPath p : paths.values()) {
				if (p.name.contains("R")) {
					int index = p.name.indexOf("R");
					if (index <= 2) {
						firstPathChooser.addOption(p.name, p);
					}
				}
				if (p.name.contains("M")) {
					int index = p.name.indexOf("M");
					if (index <= 2) {
						firstPathChooser.addOption(p.name, p);
					}
				}
				if (p.name.contains("L")) {
					int index = p.name.indexOf("L");
					if (index <= 2) {
						firstPathChooser.addOption(p.name, p);
					}
				}
				if (p.name.contains("Feed")) {
					int index = p.name.indexOf("e");
					if (index <= 2) {
						secondPathChooser.addOption(p.name, p);
						thirdPathChooser.addOption(p.name, p);
					}
				}

				if (p.name.contains("C")) {
					int index = p.name.indexOf("C");
					if (index <= 2) {
						String substring;
						if (p.name.substring(index + 1, index + 3).contains(" ")) {
							substring = p.name.substring(index + 1, index + 2);
						} else {
							substring = p.name.substring(index + 1, index + 3);
						}
						try {
							int num = Integer.parseInt(substring);
							SmartDashboard.putNumber("parsed int", num);
							if ((num % 2) == 0) {
								coralPosesRight.put(p.getStartingHolonomicPose().get(), p.name);
							} else {
								coralPosesLeft.put(p.getStartingHolonomicPose().get(), p.name);
							}
						} catch (Exception e) {
							DriverStation.reportError(e.getMessage(), e.getStackTrace());
						}
					}
				}
			}
			pathsHaveInit = true;
		}

		public static void flipCoralPoses() {

			HashMap<Pose2d, String> flippedPosesRight = new HashMap<>();
			for (Pose2d p : coralPosesRight.keySet()) {

				flippedPosesRight.put(FlippingUtil.flipFieldPose(p), coralPosesRight.get(p));
			}
			coralPosesRight.clear();
			coralPosesRight = flippedPosesRight;

			HashMap<Pose2d, String> flippedPosesLeft = new HashMap<>();
			for (Pose2d p : coralPosesLeft.keySet()) {
				flippedPosesLeft.put(FlippingUtil.flipFieldPose(p), coralPosesLeft.get(p));
			}
			coralPosesLeft.clear();
			coralPosesLeft = flippedPosesLeft;
		}

		public static PathPlannerPath lastPathFirst = Paths.firstPathChooser.get();
		public static PathPlannerPath lastPathSecond = Paths.secondPathChooser.get();
		public static PathPlannerPath lastPathThird = Paths.thirdPathChooser.get();
		public static ElevatorPos lastPosFirst = firstElevatorPosChooser.get();
		public static ElevatorPos lastPosSecond = secondElevatorPosChooser.get();
		public static ElevatorPos lastPosThird = thirdElevatorPosChooser.get();
		public static ElevatorPos lastPosFourth = fourthElevatorPosChooser.get();

		public static boolean recompileNeeded() {
			return (Paths.firstPathChooser.get() != lastPathFirst
					|| Paths.secondPathChooser.get() != lastPathSecond
					|| Paths.thirdPathChooser.get() != lastPathThird
					|| Paths.firstElevatorPosChooser.get() != lastPosFirst
					|| Paths.secondElevatorPosChooser.get() != lastPosSecond
					|| Paths.thirdElevatorPosChooser.get() != lastPosThird
					|| Paths.fourthElevatorPosChooser.get() != lastPosFourth);
		}

		public static void updateStoredChooser() {
			lastPathFirst = Paths.firstPathChooser.get();
			lastPathSecond = Paths.secondPathChooser.get();
			lastPathThird = Paths.thirdPathChooser.get();
			lastPosFirst = firstElevatorPosChooser.get();
			lastPosSecond = secondElevatorPosChooser.get();
			lastPosThird = thirdElevatorPosChooser.get();
			lastPosFourth = fourthElevatorPosChooser.get();
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

		public static final PathPlannerPath driveForwardSlow = getPathFromFile("Drive forward slow");
	}
}
