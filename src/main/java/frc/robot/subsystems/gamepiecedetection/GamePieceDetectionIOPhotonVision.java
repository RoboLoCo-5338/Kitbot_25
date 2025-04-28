package frc.robot.subsystems.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedList;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class GamePieceDetectionIOPhotonVision implements GamePieceDetectionIO{
    protected final PhotonCamera camera;
    protected final Supplier<Pose2d> robotPose;
    protected final Transform3d robotToCamera;

    public GamePieceDetectionIOPhotonVision(String name, GamePieceType detectedType, Supplier<Pose2d> robotPose, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotPose = robotPose;
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs){
        LinkedList<GamePiece> gamePieces = new LinkedList<>();
        inputs.connected = camera.isConnected();
        for (PhotonPipelineResult result: camera.getAllUnreadResults()){
            for(PhotonTrackedTarget target: result.getTargets()){
                //TODO: Figure out what targetHeight and targetPitch should be for game pieces
                PhotonUtils.estimateCameraToTargetTranslation(PhotonUtils.calculateDistanceToTargetMeters(Constants.FLOOR_TO_MECHANISM+robotToCamera.getZ(), 0, robotToCamera.getRotation().getY(), 0), Rotation2d.fromRadians(-target.getYaw()));
            }
        }
    }
}
