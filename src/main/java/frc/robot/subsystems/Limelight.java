package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Limelight extends SubsystemBase {
	private static Limelight instance = null;

	public static Limelight getInstance() {

		if (instance == null) {
			instance = new Limelight();
		}
		return instance;
	}

	private PhotonCamera camera = new PhotonCamera(Constants.Limelight.photonCamName);
	private PhotonPipelineResult result = camera.getLatestResult();
	private double lastYaw;
	private double lastDistance;
	private double lastPitch;

	public Limelight() {
		// might have to put some nt initilizations in here...
	}

	public boolean hasTarget() {
		return result.hasTargets();
	}

	public PhotonTrackedTarget getBestTarget() {
		if (result.hasTargets()) {
			if (result.getBestTarget().getFiducialId() > 0
					&& result.getBestTarget().getFiducialId() < Constants.Limelight.gameAprilTags.length) {
				return result.getBestTarget();
			}
		}
		return null;
	}

	public double getYaw() {
		if (getBestTarget() != null) {
			lastYaw = getBestTarget().getYaw();
		}
		return lastYaw;
	}

	public double getPitch() {
		if (getBestTarget() != null) {
			lastPitch = getBestTarget().getPitch();
		}
		return lastPitch;
	}

	public int getID() {
		return getBestTarget().getFiducialId();
	}

	public double getDistance() {
		if (getBestTarget() != null) {
			lastDistance = PhotonUtils.calculateDistanceToTargetMeters(
					Constants.Limelight.cameraOffsets.getZ(),
					Constants.Limelight.gameAprilTags[getBestTarget().getFiducialId() - 1].getZ(),
					Constants.Limelight.cameraAngleOffsets.getY(),
					Units.degreesToRadians(getBestTarget().getPitch()));
		}
		return lastDistance;
	}

	@Override
	public void periodic() {
		result = camera.getLatestResult();
		SmartDashboard.putBoolean("PV has target", hasTarget());
		SmartDashboard.putNumber("PV last yaw", getYaw());
		SmartDashboard.putNumber("PV last pitch", getPitch());
		SmartDashboard.putNumber("PV last distance", getDistance());
	}
}
