package frc.robot.subsystems.roller;

public class RollerIOTalonSRX implements RollerIO {

  // private final StatusSignal<AngularVelocity> rollerVelocity;
  // private final StatusSignal<Voltage> rollerAppliedVolts;
  // private final StatusSignal<Current> rollerCurrent;
  // private final StatusSignal<Temperature> rollerTemperature;

  // private final Debouncer effectorDebouncer = new Debouncer(0.5);

  public RollerIOTalonSRX() {

    // rollerVelocity = rollerMotor.getVelocity();
    // rollerAppliedVolts = rollerMotor.getMotorVoltage();
    // rollerCurrent = rollerMotor.getStatorCurrent();
    // rollerTemperature = rollerMotor.getDeviceTemp();

    // rollerMotor.getConfigurator().apply(getRollerConfiguration());

    // tryUntilOk(
    //     5,
    //     () ->
    //         BaseStatusSignal.setUpdateFrequencyForAll(
    //             50.0, rollerVelocity, rollerAppliedVolts, rollerCurrent));

    // ParentDevice.optimizeBusUtilizationForAll(rollerMotor);
  }

  // @Override
  // public void updateInputs(RollerIOInputs inputs) {
  //   var motor1Status =
  //       BaseStatusSignal.refreshAll(rollerVelocity, rollerCurrent, rollerAppliedVolts);

  //   inputs.rollerConnected = effectorDebouncer.calculate(motor1Status.isOK());
  //   inputs.rollerVelocity = Units.rotationsToRadians(rollerVelocity.getValueAsDouble());
  //   inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
  //   inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
  //   inputs.rollerTemperature = rollerTemperature.getValueAsDouble();
  // }

  // @Override
  // public void setRollerVelocity(double velocity) {
  //   rollerMotor.setControl(
  //       rollerVelocityRequest.withVelocity(velocity * EndEffectorConstants.GEARING));
  // }

  // @Override
  // public void setRollerSpeed(double speed) {
  //   rollerMotor.set(speed);
  // }
}
