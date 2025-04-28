package frc.robot.subsystems.gamepiecedetection;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.TargetObservation;
import org.littletonrobotics.junction.AutoLog;

public interface GamePieceDetectionIO {
  @AutoLog
  public static class GamePieceDetectionIOInputs {
    public boolean connected = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(new Rotation2d(), new Rotation2d());

    public GamePiece[] gamePieces = new GamePiece[0];
  }
}
