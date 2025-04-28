package frc.robot.subsystems.gamepiecedetection;
import frc.robot.subsystems.gamepiecedetection.GamePieceDetectionIOInputsAutoLogged;
public class GamePieceDetection {
  private final GamePieceDetectionIO io;
  private final GamePieceDetectionIOInputsAutoLogged inputs =
      new GamePieceDetectionIOInputsAutoLogged();

  public GamePieceDetection(GamePieceDetectionIO io) {
    this.io = io;
  }
}
