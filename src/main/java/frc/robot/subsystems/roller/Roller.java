package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.Logger;

public class Roller extends SubsystemBase {

  public final RollerIO io;
  private final RollerIOInputsAutoLogged inputs = new RollerIOInputsAutoLogged();

  private final Alert rollerDisconnectedAlert =
      new Alert("Roller motor disconnected!!", AlertType.kError);

  public Roller(RollerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Roller", inputs);

    rollerDisconnectedAlert.set(!inputs.rollerConnected && Constants.currentMode != Mode.SIM);
  }

  public Command setRollerVelocity(double velocity) {
    return new InstantCommand(
        () -> {
          io.setRollerVelocity(velocity);
        },
        this);
  }

  public Command setRollerSpeed(double speed) {
    return new InstantCommand(() -> io.setRollerSpeed(speed));
  }

  public RollerIO getIO() {
    return io;
  }
}
