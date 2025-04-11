package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj.Preferences;

public final class RollerConstants {
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
