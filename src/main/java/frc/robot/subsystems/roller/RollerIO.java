package frc.robot.subsystems.roller;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  public final TalonSRX rollerMotor = new TalonSRX(RollerConstants.ROLLER_MOTOR_ID);
  final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0.0);

  @AutoLog
  public static class RollerIOInputs {
    public double rollerVelocity = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerCurrentAmps = 0.0;
    public boolean rollerConnected = false;
    public double rollerDistance1 = -1;
    public double rollerDistance2 = -1;
    public double rollerTemperature = 0.0;
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setRollerVelocity(double velocity) {}

  public default TalonFXConfiguration getRollerConfiguration() {
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Slot0.kP = RollerConstants.ROLLER_KP;
    config.Slot0.kI = RollerConstants.ROLLER_KI;
    config.Slot0.kD = RollerConstants.ROLLER_KD;
    config.Slot0.kG = RollerConstants.ROLLER_KG;
    config.Slot0.kV = RollerConstants.ROLLER_KV;

    var currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimit = RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT;
    config.CurrentLimits = currentConfig;
    return config;
  }

  public default void setRollerSpeed(double speed) {}
}
