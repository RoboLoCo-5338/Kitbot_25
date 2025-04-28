package frc.robot.subsystems.gamepiecedetection;

import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import org.littletonrobotics.junction.AutoLog;

public interface GamePieceDetectionIO {
  @AutoLog
  public static class GamePieceDetectionIOInputs {
    public boolean connected = false;
    public GamePiece[] gamePieces = new GamePiece[0];
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
