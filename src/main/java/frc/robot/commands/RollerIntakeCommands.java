package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CANRollerSubsystem
;
public class RollerIntakeCommands {
    public static Command intakeInside() {
        return new InstantCommand(() -> RobotContainer.m_Intake.takeIntake());
    }
    public static Command intakeOutside() {
        return new InstantCommand(() -> RobotContainer.m_Intake.takeOuttake());
    }

    
}
