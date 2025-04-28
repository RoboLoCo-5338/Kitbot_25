package frc.robot.subsystems.vision.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePiece;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePieceType;
import java.util.LinkedList;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class GamePieceDetectionIOPhotonVision implements GamePieceDetectionIO {
  protected final PhotonCamera camera;
  protected final Supplier<Pose2d> robotPose;
  protected final Transform3d robotToCamera;
  protected final GamePieceType detectedType;

  public GamePieceDetectionIOPhotonVision(
      String name,
      GamePieceType detectedType,
      Supplier<Pose2d> robotPose,
      Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotPose = robotPose;
    this.robotToCamera = robotToCamera;
    this.detectedType = null;
  }

  @Override
  public void updateInputs(GamePieceDetectionIOInputs inputs) {
    LinkedList<GamePiece> gamePieces = new LinkedList<>();
    inputs.connected = camera.isConnected();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        Pose3d targetPose =
            new Pose3d(robotPose.get()).plus(robotToCamera).plus(target.getBestCameraToTarget());
        gamePieces.add(
            new GamePiece(
                result.getTimestampSeconds(), detectedType, targetPose, target.poseAmbiguity));
      }
    }
    inputs.gamePieces = new GamePiece[gamePieces.size()];
    for (int i = 0; i < gamePieces.size(); i++) {
      inputs.gamePieces[i] = gamePieces.get(i);
    }
  }
}
