package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final int rollerCanId = 0;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final double ROBOT_LENGTH = Units.inchesToMeters(33.250000);
  public static final double FLOOR_TO_MECHANISM = Units.inchesToMeters(8);

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
