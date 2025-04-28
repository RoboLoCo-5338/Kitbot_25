package frc.robot.subsystems.gamepiecedetection;

public class GamePieceDetection {
  private final GamePieceDetectionIO io;
  private final GamePieceDetectionIOInputsAutoLogged inputs =
      new GamePieceDetectionIOInputsAutoLogged();

  public GamePieceDetection(GamePieceDetectionIO io) {
    this.io = io;
  }
}
