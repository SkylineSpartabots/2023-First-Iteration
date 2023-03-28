/*
 limelight subsystem, encapsulates methods to use the limelight
*/

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

	private PhotonCamera camera;
	private PhotonPipelineResult result;
	private double lastYaw;
	private double lastDistance;
	private double lastPitch;
	private int lastID;
	private PhotonTrackedTarget lastTarget;

	public Limelight() {
		camera = new PhotonCamera(Constants.Limelight.photonCamName);
		result = camera.getLatestResult();
	}

	public boolean hasTarget() {
		if (result.hasTargets()) {
			if (result.getBestTarget().getFiducialId() > 0 
			&& result.getBestTarget().getFiducialId() < Constants.Limelight.gameAprilTags.length + 1) {
				return true;
			}
		}
		return false;
	}

	// gets the target that the limelight detects
	// has information about target distance and position relative to robot
	public PhotonTrackedTarget getBestTarget() {
		if (hasTarget()) {
			lastTarget = result.getBestTarget();
		}
		return lastTarget;
	}

	// these methods are all pretty straighforward to understand
    // they do what their name suggests

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
		getBestTarget();
		SmartDashboard.putBoolean("cam has target", hasTarget());
		SmartDashboard.putNumber("Tag id", getID());
		// SmartDashboard.putNumber("cam last yaw", getYaw());
		// SmartDashboard.putNumber("cam last pitch", getPitch());
		// SmartDashboard.putNumber("cam last distance", getDistance());
		
		// SmartDashboard.putNumber("Tag x-pos", Constants.Limelight.gameAprilTags2d[getID()-1].getX());
		// SmartDashboard.putNumber("Tag y-pos", Constants.Limelight.gameAprilTags2d[getID()-1].getY());
	}
}
