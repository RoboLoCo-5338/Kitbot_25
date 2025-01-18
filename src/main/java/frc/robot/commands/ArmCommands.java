// package frc.robot.commands;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import frc.robot.RobotContainer;

// public class ArmCommands {
// public static Command MoveArmUpCommand() {
// return new RunCommand(
// () -> RobotContainer.m_arm.moveUp(),
// RobotContainer.m_arm);
// }

// public static Command MoveArmDownCommand() {
// return new RunCommand(
// () -> RobotContainer.m_arm.moveDown(),
// RobotContainer.m_arm);
// }

// public static Command MoveArmUpSlowCommand() {
// return new RunCommand(
// () -> RobotContainer.m_arm.moveUpSlow(),
// RobotContainer.m_arm);
// }

// public static Command MoveArmDownSlowCommand() {
// return new RunCommand(
// () -> RobotContainer.m_arm.moveDownSlow(),
// RobotContainer.m_arm);
// }

// public static Command setArmCommand(double angle) {
// return new RunCommand(
// () -> RobotContainer.m_arm.setAngle(angle),
// RobotContainer.m_arm);
// }

// public static Command stopArm() {
// return new InstantCommand(
// () -> RobotContainer.m_arm.stopMotor(), RobotContainer.m_arm);
// }

// public static Command changeOffset() {
// return new InstantCommand(
// () -> RobotContainer.m_arm.changeOffset(), RobotContainer.m_arm);
// }

// public static Command getPositionCommand() {
// return new InstantCommand(
// () -> RobotContainer.m_arm.getPosition(), RobotContainer.m_arm);
// }

// public static Command setTargetPositionCommand(double targetPosition) {
// return new InstantCommand(
// () -> RobotContainer.m_arm.setTargetPosition(targetPosition),
// RobotContainer.m_arm);
// }

// public static Command movearmPosition(double targetPosition) {
// return new FunctionalCommand(() -> {
// SmartDashboard.putString("Arm Command", "Started2");
// },
// () -> RobotContainer.m_arm.setTargetPosition(targetPosition), interrupted ->
// {
// RobotContainer.m_arm.stopMotor();
// SmartDashboard.putString("Arm Command", "ended2");
// },
// () -> Math.abs(targetPosition - RobotContainer.m_arm.getPosition()) < 5,
// RobotContainer.m_arm);
// }

// }
