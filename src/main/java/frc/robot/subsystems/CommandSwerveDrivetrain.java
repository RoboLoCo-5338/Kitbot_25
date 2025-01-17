package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends LegacySwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean hasAppliedOperatorPerspective = false;

	// added this - rohit
	private final LegacySwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new LegacySwerveRequest.SysIdSwerveTranslation();
	private final LegacySwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new LegacySwerveRequest.SysIdSwerveSteerGains();
	private final LegacySwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new LegacySwerveRequest.SysIdSwerveRotation();

	private final LegacySwerveRequest.ApplyChassisSpeeds AutoRequest = new LegacySwerveRequest.ApplyChassisSpeeds();

	/*
	 * SysId routine for characterizing translation. This is used to find PID gains
	 * for the drive motors.
	 */
	private final SysIdRoutine m_SysIdRoutineTranslation = new SysIdRoutine(new SysIdRoutine.Config(null, // ramp rate
																											// (default
																											// 1 V/s)
			Volt.of(4), // dynamic step voltage
			null, // default timeout 10 second
			state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
			new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null,
					this));

	/*
	 * SysId routine for characterizing steer. This is used to find PID gains for
	 * the steer motors.
	 */
	private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp
																									// rate (1 V/s)
			Units.Volts.of(7), null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
			new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

	/*
	 * SysId routine for characterizing rotation. This is used to find PID gains for
	 * the FieldCentricFacingAngle HeadingController. See the documentation of
	 * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
	 */
	private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
			/* This is in radians per second², but SysId only supports "volts per second" */
			Units.Volts.of(Math.PI / 6).per(Units.Second),
			/* This is in radians per second, but SysId only supports "volts" */
			Units.Volts.of(Math.PI), null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
			new SysIdRoutine.Mechanism((volts) -> setControl(m_rotationCharacterization.withVolts(volts)), null, this));

	/* Change this to the sysid routine you want to test */
	private final SysIdRoutine RoutineToApply = m_SysIdRoutineTranslation;

	public CommandSwerveDrivetrain(LegacySwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
			LegacySwerveModuleConstants... modules) {
		super(driveTrainConstants, OdometryUpdateFrequency, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
		// configurePathPlanner();
	}

	public CommandSwerveDrivetrain(LegacySwerveDrivetrainConstants driveTrainConstants,
			LegacySwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
		// configurePathPlanner();

	}

	// private void configurePathPlanner() {
	// double driveBaseRadius = 0;
	// for (var moduleLocation : m_moduleLocations) {
	// driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
	// }

	// AutoBuilder.configureHolonomic(
	// ()->this.getState().Pose, // Supplier of current robot pose
	// this::seedFieldRelative, // Consumer for seeding pose against auto
	// this::getCurrentRobotChassisSpeeds,
	// (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Consumer of
	// ChassisSpeeds to drive the robot
	// new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
	// new PIDConstants(10, 0, 0),
	// TunerConstants.kSpeedAt12VoltsMps,
	// driveBaseRadius,
	// new ReplanningConfig()),
	// () -> DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Red, //
	// Assume the path needs to be flipped for Red vs Blue, this is normally the
	// case
	// this); // Subsystem for requirements
	// }

	public Command applyRequest(Supplier<LegacySwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	public void drive(double xSpeed, double ySpeed, double rotSpeed,
			Supplier<LegacySwerveRequest.FieldCentric> requestSupplier) {
		this.setControl(requestSupplier.get().withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rotSpeed));
	}
	/*
	 * Both the sysid commands are specific to one particular sysid routine, change
	 * which one you're trying to characterize
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return RoutineToApply.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return RoutineToApply.dynamic(direction);
	}

	public ChassisSpeeds getCurrentRobotChassisSpeeds() {
		return m_kinematics.toChassisSpeeds(getState().ModuleStates);
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}

	@Override
	public void periodic() {
		/* Periodically try to apply the operator perspective */
		/*
		 * If we haven't applied the operator perspective before, then we should apply
		 * it regardless of DS state
		 */
		/*
		 * This allows us to correct the perspective in case the robot code restarts
		 * mid-match
		 */
		/*
		 * Otherwise, only check and apply the operator perspective if the DS is
		 * disabled
		 */
		/*
		 * This ensures driving behavior doesn't change until an explicit disable event
		 * occurs during testing
		 */
		if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent((allianceColor) -> {
				this.setOperatorPerspectiveForward(allianceColor == Alliance.Red
						? RedAlliancePerspectiveRotation
						: BlueAlliancePerspectiveRotation);
				hasAppliedOperatorPerspective = true;
			});
		}
	}

	// everything after this is stuff i added - Rohit

}
