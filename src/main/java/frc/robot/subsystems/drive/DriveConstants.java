package frc.robot.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class DriveConstants {
  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  public static final double ROBOT_MASS_KG = 50.088;
  public static final double ROBOT_MOI = 6.883;
  public static final double WHEEL_COF = 0.975;

  static final Lock odometryLock = new ReentrantLock();
  public static final double DEADBAND = 0.06;
  public static final double ANGLE_KP = 5.0;
  public static final double ANGLE_KD = 0.4;
  public static final double ANGLE_MAX_VELOCITY = 8.0;
  public static final double ANGLE_MAX_ACCELERATION = 20.0;
  public static final double FF_START_DELAY = 2.0; // Secs
  public static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  public static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  public static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2
  public static final double AUTO_ALIGN_X_TOLERANCE = 0.005;
  public static final double AUTO_ALIGN_Y_TOLERANCE = 0.005;
  public static final double AUTO_ALIGN_ANGULAR_TOLERANCE = 0.005;

  public static final double AUTO_ALIGN_DRIVE_KP = 2;
  public static final double AUTO_ALIGN_DRIVE_KI = 0.2;
  public static final double AUTO_ALIGN_DRIVE_KD = 0.0;
  public static final double AUTO_ALIGN_TURN_KP = 2;
  public static final double AUTO_ALIGN_TURN_KI = 0.2;
  public static final double AUTO_ALIGN_TURN_KD = 0.0;

  public static final Pose2d reefRight =
      new Pose2d(3.06 - Units.inchesToMeters(2.5), 3.77 + 0.05 + 0.0127, new Rotation2d());
  public static final Pose2d reefLeft =
      new Pose2d(3.06 - Units.inchesToMeters(2.5), 4.175, new Rotation2d());
  public static final Pose2d reefCenter =
      new Pose2d(3.06 - Units.inchesToMeters(2.5), 3.8327, new Rotation2d());

  public final class DriveSimConstants {
    public static final double DRIVE_KP = 0.05;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV_ROT =
        0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    public static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    public static final double TURN_KP = 8.0;
    public static final double TURN_KD = 0.0;
    public static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    public static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);
  }
}
