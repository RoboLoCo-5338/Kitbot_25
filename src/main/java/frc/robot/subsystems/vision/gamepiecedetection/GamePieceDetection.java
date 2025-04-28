package frc.robot.subsystems.vision.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConstants;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class GamePieceDetection extends SubsystemBase {

  private final GamePieceDetectionIO[] io;
  private final GamePieceDetectionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  public GamePieceDetection(GamePieceDetectionIO... io) {
    this.io = io;
    this.inputs = new GamePieceDetectionIOInputsAutoLogged[io.length];

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }
    List<Pose3d> allGamePiecePoses = new LinkedList<>();
    List<Pose3d> allGamePiecePosesAccepted = new LinkedList<>();

    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      List<Pose3d> gamePiecePoses = new LinkedList<>();

      for (GamePiece gamePiece : inputs[cameraIndex].gamePieces) {
        gamePiecePoses.add(gamePiece.pose);
        if (gamePiece.ambiguity < VisionConstants.maxGamePieceDetectionAmbiguity) {
          boolean alreadyExists = false;
        }
      }
    }
  }

  private static double getMagnitude(Transform3d transform3d) {
    return Math.sqrt(
        Math.pow(transform3d.getX(), 2)
            + Math.pow(transform3d.getY(), 2)
            + Math.pow(transform3d.getZ(), 2));
  }

  public static record GamePiece(
      double timestamp, GamePieceType type, Pose3d pose, double ambiguity) {}

  enum GamePieceType {
    Coral,
    Algae
  }
}
