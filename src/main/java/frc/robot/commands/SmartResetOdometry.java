package frc.robot.commands;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartResetOdometry extends CommandBase {
    int kTargetPitch = 0;
    Swerve s_Swerve;
    Limelight s_Limelight;
    boolean isReset;

    public SmartResetOdometry() {
        s_Swerve = Swerve.getInstance();
        addRequirements(s_Swerve);
        s_Limelight = Limelight.getInstance();
        addRequirements(s_Limelight);
    }
    
    @Override
    public void initialize() {
        isReset = false;

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
            SmartDashboard.putNumber("robot-SO-x",  /*Units.metersToInches*/(robotPose.getX()));
            SmartDashboard.putNumber("robot-SO-y", /*Units.metersToInches*/(robotPose.getY()));
            SmartDashboard.putNumber("robot-SO-z", /*Units.metersToInches*/(robotPose.getZ()));
            SmartDashboard.putNumber("robot-SO-rot", Units.radiansToDegrees(robotPose.getRotation().getZ()));
            s_Swerve.resetOdometry(new Pose2d(robotPose.getX(), robotPose.getY(),
                    Rotation2d.fromRadians(robotPose.getRotation().getZ())));
            isReset = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isReset;
    }
    
    @Override
    public void end(boolean interrupted){
        //code for targetDestination pose2d.
    }
}
