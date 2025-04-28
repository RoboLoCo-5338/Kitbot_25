package frc.robot.subsystems.vision.gamepiecedetection;

import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePiece;
import org.littletonrobotics.junction.AutoLog;

public interface GamePieceDetectionIO {
  @AutoLog
  public static class GamePieceDetectionIOInputs {
    public boolean connected = false;
    public GamePiece[] gamePieces = new GamePiece[0];
  }

  public default void updateInputs(GamePieceDetectionIOInputs inputs) {}
}
