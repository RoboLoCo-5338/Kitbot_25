package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.VisionConstants;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.Pigeon2;

public class Vision {
	private PhotonCamera camera;
	public Vision(String cameraName) {
		camera = new PhotonCamera("Rock");
	}

	public boolean hasResults() {
		return camera.getLatestResult().hasTargets();
	}

	public double getTargetYaw() {
		if (hasResults())
			return camera.getLatestResult().getBestTarget().getYaw();
		return 0;
	}

	public double getTargetPitch() {
		if (hasResults())
			return camera.getLatestResult().getBestTarget().getPitch();
		return 0;
	}

	public double getTargetYaw(int aprilTag) {
		Optional<PhotonTrackedTarget> tag = getTagTarget(aprilTag);
		if (tag.isPresent())
			return tag.get().getYaw();
		return 0;
	}

	public double getTargetPitch(int aprilTag) {
		Optional<PhotonTrackedTarget> tag = getTagTarget(aprilTag);
		if (tag.isPresent())
			return tag.get().getYaw();
		return 0;
	}

	public double getDistanceToTarget(double targetHeight) {
		if (hasResults()) {
			return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS, targetHeight,
					VisionConstants.CAMERA_PITCH_RADIANS, getTargetPitch());
		}
		return -1;
	}

	public double getDistanceToTarget(int aprilTag) {
		if (getTagTarget(aprilTag).isPresent()) {
			return PhotonUtils.calculateDistanceToTargetMeters(VisionConstants.CAMERA_HEIGHT_METERS,
					VisionConstants.kTagLayout.getTagPose(aprilTag).get().getY(), VisionConstants.CAMERA_PITCH_RADIANS,
					getTargetPitch(aprilTag));
		}
		return -1;
	}

	public int getBestTargetID() {
		if (hasResults())
			return camera.getLatestResult().getBestTarget().getFiducialId();
		return -1;
	}

	public Optional<PhotonTrackedTarget> getTagTarget(int tag) {
		if (hasResults())
			for (PhotonTrackedTarget target : camera.getLatestResult().getTargets())
				if (target.getFiducialId() == tag)
					return Optional.of(target);
		return Optional.empty();
	}

	public Transform3d getPose() { // TODO: Maybe swap this with the other pose estimator
		var result = camera.getLatestResult();
		if (result.getMultiTagResult().isPresent()) {
			return result.getMultiTagResult().get().estimatedPose.best;
		}
		return null;
	}

	public void updateGyro(Pigeon2 gyro) {
		gyro.setYaw(getPose().getRotation().getAngle());
	}
}
