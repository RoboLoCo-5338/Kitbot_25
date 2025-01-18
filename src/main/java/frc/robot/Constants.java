package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;

public class Constants {
	public static final double scorePreset = 85;
	public static final double sourcePreset = 0;
	public static final double groundPreset = 115;
	public static final double stackPreset = 110;
	public static final class DriveConstants {
		public static final double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
		// originally 1.5 radians per second
		public static final double MaxAngularRate = 1.0 * Math.PI; // 3/4 of a rotation per second max angular velocity
	}
	public static final class RollerConstants {
		public static final int ROLLER_MOTOR_ID = 25;
		public static final double rollerMotorSpeed = 0.35;
		// public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
		// public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
		// public static final double ROLLER_EJECT_VALUE = 0.44;
	}
	public static final class VisionConstants {
		public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
		public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(42);
		public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
	}
	public static final class OIConstants {
		public static final double kDriveDeadband = 0.10;
	}
}
