package frc.robot.subsystems.vision.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.DetectionType;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePiece;
import frc.robot.subsystems.vision.gamepiecedetection.GamePieceDetection.GamePieceType;
import java.util.LinkedList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public abstract class GamePieceDetectionIOPhotonVision implements GamePieceDetectionIO {
  protected final PhotonCamera camera;
  protected final Supplier<Pose2d> robotPose;
  protected final Transform3d robotToCamera;
  protected final DetectionType detectionType;

  public GamePieceDetectionIOPhotonVision(
      String name,
      Supplier<Pose2d> robotPose,
      Transform3d robotToCamera,
      DetectionType detectionType) {
    camera = new PhotonCamera(name);
    this.robotPose = robotPose;
    this.robotToCamera = robotToCamera;
    this.detectionType = detectionType;
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
                result.getTimestampSeconds(),
                getDetectedType(target),
                targetPose,
                target.poseAmbiguity,
                detectionType,
                getConfidence(target)));
      }
    }
    inputs.gamePieces = new GamePiece[gamePieces.size()];
    for (int i = 0; i < gamePieces.size(); i++) {
      inputs.gamePieces[i] = gamePieces.get(i);
    }
    Logger.recordOutput(
        "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
        robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
  }

  public abstract GamePieceType getDetectedType(PhotonTrackedTarget target);

  public abstract float getConfidence(PhotonTrackedTarget target);
}
