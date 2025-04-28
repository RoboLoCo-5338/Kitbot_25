package frc.robot.subsystems.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;
import java.util.LinkedList;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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
  public void updateInputs(VisionIOInputs inputs) {
    LinkedList<GamePiece> gamePieces = new LinkedList<>();
    inputs.connected = camera.isConnected();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      for (PhotonTrackedTarget target : result.getTargets()) {
        // TODO: Figure out what targetHeight and targetPitch should be for game pieces
        //Calculates the translation from the camera to the target
        Translation2d cameraToTarget = PhotonUtils.estimateCameraToTargetTranslation(
            PhotonUtils.calculateDistanceToTargetMeters(
                Constants.FLOOR_TO_MECHANISM + robotToCamera.getZ(),
                0,
                robotToCamera.getRotation().getY(),
                0),
            Rotation2d.fromRadians(-target.getYaw()));
          //Transforms the robot's pose by the camera's pose and then transforms it by the camera to target translation, therefore getting the pose of the target
          Pose2d targetPose = robotPose.get().plus(new Transform2d(robotToCamera.getX(), robotToCamera.getY(), new Rotation2d(robotToCamera.getRotation().getX()))).plus(new Transform2d(cameraToTarget, robotPose.get().times(-1).getRotation()));
      }
    }
  }
}
