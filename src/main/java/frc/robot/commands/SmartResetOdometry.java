package frc.robot.commands;

import java.util.ArrayList;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartResetOdometry extends CommandBase {
    Swerve s_Swerve;
    Limelight s_Limelight;
    Timer timer = new Timer();
    ArrayList<Pose3d> poses = new ArrayList<Pose3d>();

    public SmartResetOdometry() {
        s_Swerve = Swerve.getInstance();
        s_Limelight = Limelight.getInstance();
    }
    
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        if (s_Limelight.hasTarget()) {
            PhotonTrackedTarget target = s_Limelight.getBestTarget();
            Pose3d targetPose = Constants.Limelight.gameAprilTags[target.getFiducialId() - 1];
            SmartDashboard.putNumber("pose-amb", target.getPoseAmbiguity());
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(), 
                targetPose,
                new Transform3d(Constants.Limelight.cameraOffsets, Constants.Limelight.cameraAngleOffsets)
            );
            if(target.getPoseAmbiguity() < 0.1) {
                poses.add(robotPose);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.2);
    }
    
    @Override
    public void end(boolean interrupted){
        timer.stop();
        if (poses.size() > 0) {
            double avgX = 0;
            double avgY = 0;
            double avgRot = 0;

            for (Pose3d pose : poses) {
                avgX += pose.getX();
                avgY += pose.getY();
                avgRot += pose.getRotation().getZ();
            }

            avgX /= poses.size();
            avgY /= poses.size();
            avgRot /= poses.size();

            s_Swerve.resetOdometry(new Pose2d(avgX, avgY, Rotation2d.fromRadians(avgRot)));
        }
    }
}