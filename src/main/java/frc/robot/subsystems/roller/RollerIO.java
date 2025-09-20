package frc.robot.subsystems.roller;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import org.littletonrobotics.junction.AutoLog;

public class RollerIO {
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

  public void updateInputs(RollerIOInputs inputs) {}

  public void setRollerVelocity(double velocity) {}

  public void setVoltage(double volts) {}

  public void setRollerSpeed(double speed) {}
}
