package frc.robot.subsystems.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;

public record GamePiece(double timestamp, GamePieceTypes types, Pose2d pose, double ambiguity) {}

enum GamePieceTypes {
  Coral,
  Algae
}
