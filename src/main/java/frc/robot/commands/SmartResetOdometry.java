package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public class SmartResetOdometry extends CommandBase {
    int kTargetPitch = 0;
    Swerve s_Swerve;
    Limelight s_Limelight;
    boolean isReset;

    public SmartResetOdometry(){
        s_Swerve = Swerve.getInstance();
        addRequirements(s_Swerve);
        s_Limelight = Limelight.getInstance();
        addRequirements(s_Limelight);
        isReset = false;
    }

    @Override
    public void initialize(){
        
    }

    @Override
    public void execute(){
        if(s_Limelight.hasTarget()){
            PhotonTrackedTarget target = s_Limelight.getBestTarget();
            Pose3d targetPose = Constants.Limelight.gameAprilTags[s_Limelight.getID()-1];
            //Pose2d robotPose = PhotonUtils.estimateFieldToRobot(Constants.limelightHeight, targetPose.getZ(), Constants.limelightPitch, Rotation2d.fromDegrees(-target.getYaw()), m_driveTrain.getGyroscopeRotation(), targetPose, new Transform2d());
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), targetPose, new Transform3d());
            // Pose2d robotPose = PhotonUtils.estimateFieldToRobot(kCameraHeight, kTargetHeight, kCameraPitch, kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose, cameraToRobot);
            s_Swerve.resetOdometry(new Pose2d(robotPose.getX(), robotPose.getY(), Rotation2d.fromDegrees(robotPose.getRotation().getZ())));
            isReset = true;
            
        }
    }

    @Override
    public boolean isFinished(){
        return isReset;
    }
}
