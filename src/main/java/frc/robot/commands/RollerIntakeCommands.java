package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class RollerIntakeCommands {
	public static Command intakeInside() {
		return new InstantCommand(() -> RobotContainer.m_Intake.takeIntake(Constants.RollerConstants.rollerMotorSpeed));
	}

	public static Command intakeOutside(double motorSpeed) {
		return new InstantCommand(() -> RobotContainer.m_Intake.takeOuttake(motorSpeed));
	}

	public static Command stopIntake() {
		return new InstantCommand(() -> RobotContainer.m_Intake.stopWheel());
	}
}
