package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

public class RollerIntakeCommands {
	public static Command intakeInside() {
		return new InstantCommand(() -> RobotContainer.m_Intake.takeIntake());
	}

	public static Command intakeOutside() {
		return new InstantCommand(() -> RobotContainer.m_Intake.takeOuttake());
	}
	public static Command stopIntake() {
		return new InstantCommand(() -> RobotContainer.m_Intake.stopWheel());
	}

}
