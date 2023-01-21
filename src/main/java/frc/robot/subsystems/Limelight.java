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
	private static Limelight instance;

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
	private int lastID;
	private PhotonTrackedTarget lastTarget;

	public Limelight() {
		// might have to put some nt initilizations in here...
	}

	public boolean hasTarget() {
		if (result.hasTargets()) {
			if (result.getBestTarget().getFiducialId() > 0 
			&& result.getBestTarget().getFiducialId() < Constants.Limelight.gameAprilTags.length) {
				return true;
			}
		}
		return false;
	}

	public PhotonTrackedTarget getBestTarget() {
		if (hasTarget()) {
			lastTarget = result.getBestTarget();
		}
		return lastTarget;
	}

	public double getYaw() {
		if (hasTarget()) {
			lastYaw = lastTarget.getYaw();
		}
		return lastYaw;
	}

	public double getPitch() {
		if (hasTarget()) {
			lastPitch = lastTarget.getPitch();
		}
		return lastPitch;
	}

	public int getID() {
		if (hasTarget()) {
			lastID = lastTarget.getFiducialId();
		}
		return lastID;
	}

	public double getDistance() {
		if (hasTarget()) {
			lastDistance = PhotonUtils.calculateDistanceToTargetMeters(
					Constants.Limelight.cameraOffsets.getZ(),
					Constants.Limelight.gameAprilTags[getID() - 1].getZ(),
					Constants.Limelight.cameraAngleOffsets.getY(),
					Units.degreesToRadians(getBestTarget().getPitch()));
		}
		return lastDistance;
	}

	@Override
	public void periodic() {
		result = camera.getLatestResult();
		SmartDashboard.putBoolean("cam has target", hasTarget());
		SmartDashboard.putNumber("cam last yaw", getYaw());
		SmartDashboard.putNumber("cam last pitch", getPitch());
		SmartDashboard.putNumber("cam last distance", getDistance());
		SmartDashboard.putNumber("Tag id", getID());
		
		// SmartDashboard.putNumber("Tag x-pos", Constants.Limelight.gameAprilTags2d[getID()-1].getX());
		// SmartDashboard.putNumber("Tag y-pos", Constants.Limelight.gameAprilTags2d[getID()-1].getY());
	}
}
