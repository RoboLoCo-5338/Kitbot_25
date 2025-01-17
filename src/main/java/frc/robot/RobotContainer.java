// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.ArmCommands;
// import frc.robot.commands.AutoCommands;
// import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.CANRollerSubsystem;
// import frc.robot.subsystems.ArmSystem;
// import frc.robot.subsystems.Intake;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.RollerIntakeCommands;

public class RobotContainer {
	// public static ArmSystem m_arm = new ArmSystem();
	// public static Intake intake = new Intake();
	private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
	// originally 1.5 radians per second
	private double MaxAngularRate = 1.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
	public static CANRollerSubsystem m_Intake = new CANRollerSubsystem();
	private boolean slow = false;
	/* Setting up bindings for necessary control of the swerve drive platform */
	public static final CommandXboxController m_driverController = new CommandXboxController(0); // driver
	public static final CommandXboxController m_operatorController = new CommandXboxController(1); // operator
	public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
	public static final Vision m_vision = new Vision("Rock");

	// public DigitalInput armLimitSwitch = new DigitalInput(9);
	public static final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric()
			.withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add
																														// a
																														// 10%
																														// deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
																		// driving in open loop
	private final LegacySwerveRequest.SwerveDriveBrake brake = new LegacySwerveRequest.SwerveDriveBrake();
	private final LegacySwerveRequest.PointWheelsAt point = new LegacySwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(MaxSpeed);

	// private final SendableChooser<Command> autoChooser;

	private static Map<String, Command> commands = new HashMap<String, Command>();

	private void configureBindings() {
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						.withVelocityX(-m_driverController.getLeftY() * MaxSpeed * (slow ? 0.3 : 1)) // Drive
						// forward
						// with
						// negative Y (forward)
						.withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-m_driverController.getRightX() * MaxAngularRate * 0.5 * (slow ? 0.3 : 1)) // Drive
				// counterclockwise
				// with
				// negative
				// X
				// (left)
				));

		m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
		m_driverController.b().whileTrue(drivetrain.applyRequest(() -> point
				.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

		// reset the field-centric heading on left bumper press
		m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		drivetrain.registerTelemetry(logger::telemeterize);

		Trigger intakeIn = new Trigger(m_operatorController.rightBumper());
		intakeIn.onTrue(RollerIntakeCommands.intakeInside());

		Trigger intakeOut = new Trigger(m_operatorController.leftBumper());
		intakeOut.onTrue(RollerIntakeCommands.intakeOutside());

		// Bindings for drivetrain characterization
		// These bindings require multiple buttons pushed to swap between quastatic
		// and dynamic
		// Back/Start select dynamic/quasistatic, Y/X select forward/reverse
		// direction
		// joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		// joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		// joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		// joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// Trigger intakeIn = new Trigger(m_operatorController.rightTrigger());
		// intakeIn.whileTrue(IntakeCommands.intake());
		// intakeIn.onFalse(IntakeCommands.stopIntake());

		// Trigger intakeOut = new Trigger(m_operatorController.leftTrigger());
		// intakeOut.whileTrue(IntakeCommands.outake());
		// intakeOut.onFalse(IntakeCommands.stopIntake());

		// Trigger ArmUp = new Trigger(m_operatorController.rightBumper());
		// ArmUp.whileTrue(ArmCommands.MoveArmUpCommand());
		// ArmUp.onFalse(ArmCommands.stopArm());

		// Trigger ArmDown = new Trigger(m_operatorController.leftBumper());
		// ArmDown.whileTrue(ArmCommands.MoveArmDownCommand());
		// ArmDown.onFalse(ArmCommands.stopArm());

		// Trigger moveArmUp = new Trigger(() -> m_operatorController.getLeftY()> 0.1);
		// moveArmUp.whileTrue(ArmCommands.MoveArmUpCommand());
		// Trigger moveArmDown = new Trigger(() -> m_operatorController.getLeftY()<
		// -0.1);
		// moveArmDown.whileTrue(ArmCommands.MoveArmDownCommand());
		// Trigger moveArmUpSlow = new Trigger(() ->
		// m_operatorController.getRightY()>0.1);
		// moveArmUpSlow.whileTrue(ArmCommands.MoveArmUpSlowCommand());
		// Trigger moveArmDownSlow = new Trigger(() ->
		// m_operatorController.getRightY()<-0.1);
		// moveArmDownSlow.whileTrue(ArmCommands.MoveArmDownSlowCommand());
		// Trigger stopArm = new Trigger(() ->
		// Math.abs(m_operatorController.getLeftY())<0.1 &&
		// Math.abs(m_operatorController.getRightY())<0.1);
		// stopArm.whileTrue(ArmCommands.stopArm());

		// Trigger armToScore = new Trigger(m_operatorController.rightBumper());
		// armToScore.onTrue(ArmCommands.setTargetPositionCommand(Constants.scorePreset));

		// Trigger armToSource = new Trigger(m_operatorController.b());
		// armToSource.onTrue(ArmCommands.setTargetPositionCommand(Constants.sourcePreset));

		// Trigger armToGround = new Trigger(m_operatorController.a());
		// armToGround.onTrue(ArmCommands.setTargetPositionCommand(Constants.groundPreset));

		Trigger slowMode = new Trigger(m_driverController.rightTrigger());
		slowMode.onTrue(new InstantCommand(() -> {
			slow = true;
		}));
		slowMode.onFalse(new InstantCommand(() -> {
			slow = false;
		}));

	}

	public RobotContainer() {
		// commands.put("StackBucket", AutoCommands.stackBucket());
		// commands.put("Score Bucket", AutoCommands.scoreBucket());
		// commands.put("Intake In", IntakeCommands.runIntakeForwardTimed(1000));
		// commands.put("Intake Out", IntakeCommands.runIntakeBackwardTimed(1000));
		// commands.put("Ground Arm",
		// ArmCommands.setTargetPositionCommand(Constants.groundPreset));
		// commands.put("Score Arm",
		// ArmCommands.setTargetPositionCommand(Constants.scorePreset));
		// commands.put("Reset Arm",
		// ArmCommands.setTargetPositionCommand(Constants.sourcePreset));
		// commands.put("Stack Arm",
		// ArmCommands.setTargetPositionCommand(Constants.stackPreset));

		NamedCommands.registerCommands(commands);
		configureBindings();

		// autoChooser = AutoBuilder.buildAutoChooser();

		// SmartDashboard.putData(autoChooser);
	}

	public Command getAutonomousCommand() {
		return null;
	}
}
