package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;

public class VisionCommands {
	public static PIDController turnController = new PIDController(0.1, 0, 0); // TODO: Update these values
	static DoubleConsumer turnBot = (rotationalSpeed) -> RobotContainer.drivetrain.drive(
			-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), OIConstants.kDriveDeadband),
			-MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), OIConstants.kDriveDeadband),
			rotationalSpeed, () -> RobotContainer.drive);

	public static Command turnToTarget() {
		turnController.enableContinuousInput(-180, 180);
		return new FunctionalCommand(() -> turnController.reset(),
				() -> turnBot.accept(turnController.calculate(
						((DoubleSupplier) (() -> RobotContainer.drivetrain.getRotation3d().getZ())).getAsDouble(),
						((DoubleSupplier) (() -> {
							if (RobotContainer.m_vision.getTargetYaw() == Double.POSITIVE_INFINITY)
								return RobotContainer.drivetrain.getRotation3d().getZ();
							return RobotContainer.drivetrain.getRotation3d().getZ()
									- RobotContainer.m_vision.getTargetYaw();
						})).getAsDouble())),
				interrupted -> turnBot.accept(0), () -> RobotContainer.m_vision.getTargetYaw() < 0.1,
				RobotContainer.drivetrain);

		// return new PIDCommand(turnController, () ->
		// RobotContainer.m_robotDrive.getHeading(),
		// // RobotContainer.m_robotDrive.getHeading() -
		// RobotContainer.m_vision.getTargetYaw() ,
		// () -> {
		// if (RobotContainer.m_vision.getTargetYaw() == Double.POSITIVE_INFINITY)
		// return RobotContainer.m_robotDrive.getHeading();
		// return RobotContainer.m_robotDrive.getHeading() -
		// RobotContainer.m_vision.getTargetYaw();
		// },
		// (rotationalSpeed) -> RobotContainer.m_robotDrive.drive(
		// -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(),
		// OIConstants.kDriveDeadband),
		// -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(),
		// OIConstants.kDriveDeadband),
		// rotationalSpeed, true, true, true, false), // TODO: Update these values
		// RobotContainer.m_robotDrive);
	}

	public static Command turnToTarget(int tagID) {
		turnController.enableContinuousInput(-180, 180);
		return new FunctionalCommand(() -> turnController.reset(),
				() -> turnBot.accept(turnController.calculate(
						((DoubleSupplier) (() -> RobotContainer.drivetrain.getRotation3d().getZ())).getAsDouble(),
						((DoubleSupplier) (() -> {
							if (RobotContainer.m_vision.getTargetYaw(tagID) == Double.POSITIVE_INFINITY)
								return RobotContainer.drivetrain.getRotation3d().getZ();
							return RobotContainer.drivetrain.getRotation3d().getZ()
									- RobotContainer.m_vision.getTargetYaw(tagID);
						})).getAsDouble())),
				interrupted -> turnBot.accept(0), () -> RobotContainer.m_vision.getTargetYaw(tagID) < 0.1,
				RobotContainer.drivetrain);
		// return new PIDCommand(turnController, () ->
		// RobotContainer.m_robotDrive.getHeading(),
		// // RobotContainer.m_robotDrive.getHeading() -
		// RobotContainer.m_vision.getTargetYaw() ,
		// () -> {
		// if (RobotContainer.m_vision.getTargetYaw() == Double.POSITIVE_INFINITY)
		// return RobotContainer.m_robotDrive.getHeading();
		// return RobotContainer.m_robotDrive.getHeading() -
		// RobotContainer.m_vision.getTargetYaw(tagId);
		// },
		// (rotationalSpeed) -> RobotContainer.m_robotDrive.drive(
		// -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(),
		// OIConstants.kDriveDeadband),
		// -MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(),
		// OIConstants.kDriveDeadband),
		// rotationalSpeed, true, true, true, false), // TODO: Update these values
		// RobotContainer.m_robotDrive);
	}
}
