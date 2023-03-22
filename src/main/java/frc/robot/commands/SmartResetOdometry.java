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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartResetOdometry extends CommandBase {
    Swerve s_Swerve;
    Limelight s_Limelight;
    Timer timer = new Timer();
    boolean isReset;
    // ArrayList<Pose3d> poses = new ArrayList<Pose3d>();

    public SmartResetOdometry() {
        s_Swerve = Swerve.getInstance();
        s_Limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // ArrayList<Pose3d> poseList = new ArrayList<>();

    @Override
    public void execute() {
        if (s_Limelight.hasTarget()) {
            PhotonTrackedTarget target = s_Limelight.getBestTarget();
            Pose3d targetPose = Constants.Limelight.gameAprilTags[target.getFiducialId() - 1];
            SmartDashboard.putNumber("pose-amb", target.getPoseAmbiguity());
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    targetPose,
                    new Transform3d(Constants.Limelight.cameraOffsets, Constants.Limelight.cameraAngleOffsets));
            double ambi = target.getPoseAmbiguity();

            if (ambi < 0.08) {
                s_Swerve.resetOdometry(new Pose2d(robotPose.getX(), robotPose.getY(),
                        Rotation2d.fromRadians(robotPose.getRotation().getZ())));
            }

            isReset = true;
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.2) || isReset;
    }

    @Override
    public void end(boolean interrupted) {
        // timer.stop();
        // if (poseList.size() > 0) {
        // double x = 0;
        // double y = 0;
        // double angle = 0;
        // for (int i = 0; i < poseList.size(); i++) {
        // x += poseList.get(i).getX();
        // y += poseList.get(i).getY();
        // angle += poseList.get(i).getRotation().getZ();
        // }

        // x /= poseList.size();
        // y /= poseList.size();
        // angle /= poseList.size();

        // s_Swerve.resetOdometry(new Pose2d(x, y, Rotation2d.fromRadians(angle)));
        // SmartDashboard.putNumber("robot-SO-x", /*Units.metersToInches*/(x));
        // SmartDashboard.putNumber("robot-SO-y", /*Units.metersToInches*/(y));
        // SmartDashboard.putNumber("robot-SO-rot", Units.radiansToDegrees(angle));
        // }
    }
}