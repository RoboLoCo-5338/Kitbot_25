// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.RobotContainer;

// public class IntakeCommands {
// private static double startTime;

// // public static Command indexForward() { indexer doesn't exist
// // return new InstantCommand(
// // () -> RobotContainer.intake.indexForward(),
// // RobotContainer.intake
// // );
// // }
// // public static Command indexReverse() { indexer doesn't exist
// // return new InstantCommand(
// // () -> RobotContainer.intake.indexReverse(),
// // RobotContainer.intake
// // );
// // }
// public static Command intake() {// creates the intake command
// return new InstantCommand(
// () -> RobotContainer.intake.intake(),
// RobotContainer.intake);
// }

// public static Command outake() {// creates the outake command
// return new InstantCommand(
// () -> RobotContainer.intake.outake(),
// RobotContainer.intake);
// }

// // public static Command stopIndex() { indexer doesn't exist
// // return new InstantCommand(
// // () -> RobotContainer.intake.stopIndex(),
// // RobotContainer.intake
// // );
// // }
// public static Command stopIntake() {// creates the stop intake command
// return new InstantCommand(
// () -> RobotContainer.intake.stopIntake(),
// RobotContainer.intake);
// }

// public static Command runIntakeForwardTimed(long time) {
// return new FunctionalCommand(() -> {
// RobotContainer.intake.stopIntake();
// startTime = System.currentTimeMillis();
// }, () -> RobotContainer.intake.intake(), interrupted ->
// RobotContainer.intake.stopIntake(),
// () -> System.currentTimeMillis() - time > startTime, RobotContainer.intake);
// }

// public static Command runIntakeBackwardTimed(long time) {
// return new FunctionalCommand(() -> {
// RobotContainer.intake.stopIntake();
// startTime = System.currentTimeMillis();
// }, () -> RobotContainer.intake.outake(), interrupted ->
// RobotContainer.intake.stopIntake(),
// () -> System.currentTimeMillis() - time > startTime, RobotContainer.intake);
// }

// }
