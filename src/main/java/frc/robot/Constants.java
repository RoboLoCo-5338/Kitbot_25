package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
	public static final double scorePreset = 85;
	public static final double sourcePreset = 0;
	public static final double groundPreset = 115;
	public static final double stackPreset = 110;

	public static final Mode simMode = Mode.SIM;
	public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,

		/** Running a physics simulator. */
		SIM,

		/** Replaying from a log file. */
		REPLAY
	}

 	public static final class RollerConstants {
		public static final int ROLLER_MOTOR_ID = 25;
		public static final double rollerMotorSpeed = 0.35;
		// public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
		// public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
		// public static final double ROLLER_EJECT_VALUE = 0.44;
	}
}
