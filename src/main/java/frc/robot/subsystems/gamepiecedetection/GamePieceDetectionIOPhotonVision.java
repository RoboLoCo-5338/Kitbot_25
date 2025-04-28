package frc.robot.subsystems.gamepiecedetection;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

public class GamePieceDetectionIOPhotonVision implements GamePieceDetectionIO {
  protected final PhotonCamera camera;
  protected final Supplier<Pose2d> robotPose;

  public GamePieceDetectionIOPhotonVision(String name, Supplier<Pose2d> robotPose) {
    camera = new PhotonCamera(name);
    this.robotPose = robotPose;
  }
}
