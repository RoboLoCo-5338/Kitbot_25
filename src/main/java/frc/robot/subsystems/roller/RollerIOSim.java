package frc.robot.subsystems.roller;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.SimMechanism;
import frc.robot.subsystems.roller.RollerConstants.RollerSimConstants;

public class RollerIOSim extends SimMechanism implements RollerIO {
  TalonSRXSimCollection simMotor = rollerMotor.getSimCollection();
  FlywheelSim physicsSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60(1), RollerSimConstants.MOI, RollerConstants.GEARING),
          DCMotor.getKrakenX60(1));

  public RollerIOSim() {
    super();
    rollerMotor.getConfigurator().apply(getRollerConfiguration());
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    simMotor.setSupplyVoltage(RobotController.getBatteryVoltage());
    physicsSim.setInputVoltage(simMotor.getMotorVoltage());

    inputs.rollerConnected = true;
    inputs.rollerVelocity = Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec());
    inputs.rollerAppliedVolts = physicsSim.getInputVoltage();
    inputs.rollerCurrentAmps = physicsSim.getCurrentDrawAmps();

    physicsSim.update(0.02);

    simMotor.addRotorPosition(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * 0.02
            * RollerConstants.GEARING);
    simMotor.setRotorVelocity(
        Units.radiansToRotations(physicsSim.getAngularVelocityRadPerSec())
            * RollerConstants.GEARING);
  }

  @Override
  public void setRollerVelocity(double velocity) {
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(velocity * RollerConstants.GEARING));
  }

  @Override
  public void setRollerSpeed(double speed) {
    System.out.println(speed);
    rollerMotor.set(speed * RollerConstants.GEARING);
  }

  @Override
  public double[] getCurrents() {
    return new double[] {physicsSim.getCurrentDrawAmps()};
  }
}
