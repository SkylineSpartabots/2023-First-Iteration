/*
 smart reset odometry command used to reset swerve odo from april tag and limelight
*/

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
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
    boolean isReset;

    public SmartResetOdometry() {
        s_Swerve = Swerve.getInstance();
        s_Limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {
        Robot.getRedPose();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // if limelight has a target then run. It gets the pose of the target that it is detecting from
        // constants and then uses the library to caluclate the robot pose in 3D which can be used to reset odo
        if (s_Limelight.hasTarget()) {
            PhotonTrackedTarget target = s_Limelight.getBestTarget();
            Pose3d targetPose = Constants.Limelight.gameAprilTags[target.getFiducialId() - 1];
            SmartDashboard.putNumber("pose-amb", target.getPoseAmbiguity());
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    targetPose,
                    new Transform3d(Constants.Limelight.cameraOffsets, Constants.Limelight.cameraAngleOffsets));
            double ambi = target.getPoseAmbiguity();

            // only reset if the target detected has an ambiguity of less than 0.08
            if (ambi < 0.08) {
                s_Swerve.resetOdometry(new Pose2d(robotPose.getX(), robotPose.getY(),
                        Rotation2d.fromRadians(robotPose.getRotation().getZ())));
            }

            isReset = true;
        }
    }

    @Override
    public boolean isFinished() {
        // timer so that it does not perpetually run if no target is detected
        return timer.hasElapsed(0.2) || isReset;
    }

    @Override
    public void end(boolean interrupted) {
    }
}