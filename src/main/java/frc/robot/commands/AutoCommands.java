// package frc.robot.commands;

// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import java.util.Optional;

// public class AutoCommands {

// 	public static PathPlannerAuto[] autos;
// 	public static String defaultAuto = "2m forward";

// 	public static void loadAutos() {
// 		// Load all autos here in this file
// 		autos = new PathPlannerAuto[1];
// 		autos[0] = new PathPlannerAuto("2m forward");
// 	}

// 	public static Pose2d getPathPose(PathPlannerPath pPath) {
// 		Optional<Alliance> alliance = DriverStation.getAlliance();
// 		if (pPath.flipPath().getStartingHolonomicPose().isPresent()) {
// 			if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
// 				return pPath.flipPath().getStartingHolonomicPose().get();
// 			}
// 			return pPath.getStartingHolonomicPose().get();
// 		}
// 		return null;
// 	}

// 	// public static Command placeCoral() {

// 	// return new SequentialCommandGroup(
// 	// new ParallelRaceGroup(ShooterCommands.runShooterBackwardTimed(100),
// 	// IntakeCommands.runIndexerOutOnlyTimed(100)),
// 	// new WaitCommand(0.25), new
// 	// ParallelCommandGroup(ShooterCommands.runShooterForwardTimed(1000),
// 	// new SequentialCommandGroup(new WaitCommand(0.75),
// 	// IntakeCommands.runIntakeForwardTimed(750))));
// 	// }
// }
