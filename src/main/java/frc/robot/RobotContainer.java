// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.Vision.VisionConstants.camera1Name;
import static frc.robot.subsystems.Vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.Vision.VisionConstants.robotToCamera1;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.commands.ArmCommands;
// import frc.robot.commands.AutoCommands;
// import frc.robot.commands.IntakeCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CANRollerSubsystem;
// import frc.robot.subsystems.ArmSystem;
// import frc.robot.subsystems.Intake;
import frc.robot.commands.RollerIntakeCommands;
import frc.robot.subsystems.Vision.*;

public class RobotContainer {
	// public static ArmSystem m_arm = new ArmSystem();
	// public static Intake intake = new Intake();
	private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
	// originally 1.5 radians per second
	private double MaxAngularRate = 1.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
	public static CANRollerSubsystem m_Intake = new CANRollerSubsystem();
	private boolean slow = false;
	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController joystick1 = new CommandXboxController(0); // driver
	private final CommandXboxController joystick2 = new CommandXboxController(1); // operator
	private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

	// public DigitalInput armLimitSwitch = new DigitalInput(9);
	private final LegacySwerveRequest.FieldCentric drive = new LegacySwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
																		// driving in open loop
	private final LegacySwerveRequest.SwerveDriveBrake brake = new LegacySwerveRequest.SwerveDriveBrake();
	private final LegacySwerveRequest.PointWheelsAt point = new LegacySwerveRequest.PointWheelsAt();

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private final SendableChooser<Command> autoChooser;

	private static Map<String, Command> commands = new HashMap<String, Command>();

	private void configureBindings() {
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-joystick1.getLeftY() * MaxSpeed * (slow ? 0.3 : 1)) // Drive
																														// forward
																														// with
						// negative Y (forward)
						.withVelocityY(-joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-joystick1.getRightX() * MaxAngularRate * 0.5 * (slow ? 0.3 : 1)) // Drive
																												// counterclockwise
																												// with
																												// negative
																												// X
																												// (left)
				));

		joystick1.a().whileTrue(drivetrain.applyRequest(() -> brake));
		joystick1.b().whileTrue(drivetrain.applyRequest(
				() -> point.withModuleDirection(new Rotation2d(-joystick1.getLeftY(), -joystick1.getLeftX()))));

		// reset the field-centric heading on left bumper press
		joystick1.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		if (Utils.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		drivetrain.registerTelemetry(logger::telemeterize);

		Trigger intakeIn = new Trigger(joystick1.rightBumper());
		intakeIn.whileTrue(RollerIntakeCommands.intakeInside(0.35));
		intakeIn.onFalse(RollerIntakeCommands.stopIntake());

		Trigger intakeOut = new Trigger(joystick1.leftBumper());
		intakeOut.whileTrue(RollerIntakeCommands.intakeOutside(0.5));
		intakeOut.onFalse(RollerIntakeCommands.stopIntake());

		Trigger slowOut = new Trigger(joystick1.leftTrigger());
		slowOut.whileTrue(RollerIntakeCommands.intakeOutside(0.2));
		slowOut.onFalse(RollerIntakeCommands.stopIntake());

		// VISION template!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!:
		// Auto aim command example
		// fix later
		// they use keyboard we use joystick

		// @SuppressWarnings("resource")
		// PIDController aimController = new PIDController(0.2, 0.0, 0.0);
		// aimController.enableContinuousInput(-Math.PI, Math.PI);
		// keyboard
		// .button(1)
		// .whileTrue(
		// Commands.startRun(
		// () -> {
		// aimController.reset();
		// },
		// () -> {
		// drive.run(0.0, aimController.calculate(vision.getTargetX(0).getRadians()));
		// },
		// drive));

		// Bindings for drivetrain characterization
		// These bindings require multiple buttons pushed to swap between quastatic
		// and dynamic
		// Back/Start select dynamic/quasistatic, Y/X select forward/reverse
		// direction
		// joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		// joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		// joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		// joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// Trigger intakeIn = new Trigger(joystick2.rightTrigger());
		// intakeIn.whileTrue(IntakeCommands.intake());
		// intakeIn.onFalse(IntakeCommands.stopIntake());

		// Trigger intakeOut = new Trigger(joystick2.leftTrigger());
		// intakeOut.whileTrue(IntakeCommands.outake());
		// intakeOut.onFalse(IntakeCommands.stopIntake());

		// Trigger ArmUp = new Trigger(joystick2.rightBumper());
		// ArmUp.whileTrue(ArmCommands.MoveArmUpCommand());
		// ArmUp.onFalse(ArmCommands.stopArm());

		// Trigger ArmDown = new Trigger(joystick2.leftBumper());
		// ArmDown.whileTrue(ArmCommands.MoveArmDownCommand());
		// ArmDown.onFalse(ArmCommands.stopArm());

		// Trigger moveArmUp = new Trigger(() -> joystick2.getLeftY()> 0.1);
		// moveArmUp.whileTrue(ArmCommands.MoveArmUpCommand());
		// Trigger moveArmDown = new Trigger(() -> joystick2.getLeftY()< -0.1);
		// moveArmDown.whileTrue(ArmCommands.MoveArmDownCommand());
		// Trigger moveArmUpSlow = new Trigger(() -> joystick2.getRightY()>0.1);
		// moveArmUpSlow.whileTrue(ArmCommands.MoveArmUpSlowCommand());
		// Trigger moveArmDownSlow = new Trigger(() -> joystick2.getRightY()<-0.1);
		// moveArmDownSlow.whileTrue(ArmCommands.MoveArmDownSlowCommand());
		// Trigger stopArm = new Trigger(() -> Math.abs(joystick2.getLeftY())<0.1 &&
		// Math.abs(joystick2.getRightY())<0.1);
		// stopArm.whileTrue(ArmCommands.stopArm());

		// Trigger armToScore = new Trigger(joystick2.rightBumper());
		// armToScore.onTrue(ArmCommands.setTargetPositionCommand(Constants.scorePreset));

		// Trigger armToSource = new Trigger(joystick2.b());
		// armToSource.onTrue(ArmCommands.setTargetPositionCommand(Constants.sourcePreset));

		// Trigger armToGround = new Trigger(joystick2.a());
		// armToGround.onTrue(ArmCommands.setTargetPositionCommand(Constants.groundPreset));

		Trigger slowMode = new Trigger(joystick1.rightTrigger());
		slowMode.onTrue(new InstantCommand(() -> {
			slow = true;
		}));
		slowMode.onFalse(new InstantCommand(() -> {
			slow = false;
		}));

	}

	private final Vision vision;

	public RobotContainer() {
		switch (Constants.currentMode) {
			case REAL :
				// Real robot, instantiate hardware IO implementations

				vision = new Vision(drivetrain::addVisionMeasurement,
						new VisionIOPhotonVision(camera0Name, robotToCamera0),
						new VisionIOPhotonVision(camera1Name, robotToCamera1));
				break;

			case SIM :
				// Sim robot, instantiate physics sim IO implementations
				vision = new Vision(drivetrain::addVisionMeasurement,
						new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drivetrain::getPose),
						new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drivetrain::getPose));
				break;

			default :
				// Replayed robot, disable IO implementations
				// (Use same number of dummy implementations as the real robot)
				vision = new Vision(drivetrain::addVisionMeasurement, new VisionIO() {
				}, new VisionIO() {
				});
				break;
		}

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
		commands.put("CoralOutake", RollerIntakeCommands.intakeOutside(0.35));

		NamedCommands.registerCommands(commands);
		configureBindings();

		autoChooser = AutoBuilder.buildAutoChooser();

		SmartDashboard.putData(autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public double getRotation2DDegrees() {
		return drivetrain.getPigeon2().getRotation2d().getDegrees();
	}

}
