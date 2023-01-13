package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class Limelight extends SubsystemBase {
	private static Limelight instance = null;

	public static Limelight getInstance() {

		if (instance == null) {
			instance = new Limelight();
		}
		return instance;
	}

	final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(35);
	final double TARGET_HEIGHT_METER = Units.inchesToMeters(104);
	final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);

	PhotonCamera camera = new PhotonCamera("gloworm");
	PhotonPipelineResult result = camera.getLatestResult();
	double lastYaw;
	double lastDistance;

	public Limelight() {
		// might have to put some nt initilizations in here...
	}

	public boolean hasTarget() {
		return result.hasTargets();
	}

	public double getYaw() {
		if (result.hasTargets()) {
			lastYaw = result.getBestTarget().getYaw();
		}
		return lastYaw;
	}

	public int getID(){
		return result.getBestTarget().getFiducialId();
	}
	
	public double getDistance() {
		if (result.hasTargets()) {
			lastDistance = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METER,
					CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
		}
		return lastDistance;
	}

	@Override
	public void periodic() {
		result = camera.getLatestResult();
		SmartDashboard.putBoolean("PV has target", hasTarget());
		SmartDashboard.putNumber("PV last yaw", getYaw());
		SmartDashboard.putNumber("PV last distance", getDistance());
	}
}
