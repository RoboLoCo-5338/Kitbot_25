package frc.robot.subsystems.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;

public record GamePiece(double timestamp, GamePieceType types, Pose2d pose, double ambiguity) {}

enum GamePieceType {
  Coral,
  Algae
}
