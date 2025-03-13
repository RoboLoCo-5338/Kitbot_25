package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

public class Constants {
	public static final double scorePreset = 85;
	public static final double sourcePreset = 0;
	public static final double groundPreset = 115;
	public static final double stackPreset = 110;

	public static final class RollerConstants {
		public static final int ROLLER_MOTOR_ID = 25;
		// public static final double rollerMotorSpeed = 0.35;
		public static double rollerMotorSpeed = Preferences.getDouble("rollerMotorSpeed", 0.0);
		// public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
		// public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
		// public static final double ROLLER_EJECT_VALUE = 0.44;
		public static void reloadConstants() {
			rollerMotorSpeed = Preferences.getDouble("rollerMotorSpeed", 0.0);
		}
	}

	public static void initPreferences() {
		String[] subsystemIdentifiers = {
		  "rollerMotor"
		};
		for (String id : subsystemIdentifiers) {
		  Preferences.initDouble(id + "Speed", 0);
		}
	  }
	
	  public static void reloadPreferences() {
		RollerConstants.reloadConstants();
	  }
}
