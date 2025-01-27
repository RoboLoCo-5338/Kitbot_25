package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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

	public class VisionConstants {
		public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

		// Camera names, must match names configured on coprocessor
		public static String camera0Name = "camera_0";
		public static String camera1Name = "camera_1";

		// Robot to camera transforms
		// (Not used by Limelight, configure in web UI instead)
		public static Transform3d robotToCamera0 = new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
		public static Transform3d robotToCamera1 = new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

		// Basic filtering thresholds
		public static double maxAmbiguity = 0.3;
		public static double maxZError = 0.75;

		// Standard deviation baselines, for 1 meter distance and 1 tag
		// (Adjusted automatically based on distance and # of tags)
		public static double linearStdDevBaseline = 0.02; // Meters
		public static double angularStdDevBaseline = 0.06; // Radians

		// Standard deviation multipliers for each camera
		// (Adjust to trust some cameras more than others)
		public static double[] cameraStdDevFactors = new double[]{1.0, // Camera 0
				1.0 // Camera 1
		};

		// Multipliers to apply for MegaTag 2 observations
		public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
		public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
	}
}
