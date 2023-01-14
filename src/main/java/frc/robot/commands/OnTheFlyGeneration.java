package frc.robot.commands;

import java.io.Console;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.subsystems.Swerve;

public class OnTheFlyGeneration extends CommandBase {
    private Pose2d currentPos;
    private Pose2d targetPos;
    private Swerve s_Swerve = Swerve.getInstance();

    public OnTheFlyGeneration(Pose2d currentPos, Pose2d targetPos) {
        this.targetPos = targetPos;
        this.currentPos = currentPos;
    }

    public OnTheFlyGeneration(Pose2d currentPos, int targetID) {
        this.currentPos = currentPos;
        this.targetPos = Constants.Limelight.gameAprilTags2d[targetID].plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
    }

    public OnTheFlyGeneration(int targetID) {
        this.currentPos = s_Swerve.getPose();
        this.targetPos = Constants.Limelight.gameAprilTags2d[targetID].plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
    }

    private PathPoint getPathPoint(Pose2d pose) {
        return new PathPoint(new Translation2d(pose.getX(), pose.getY()),
                Rotation2d.fromDegrees(0),
                pose.getRotation());
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(4, 3),
                getPathPoint(currentPos),
                getPathPoint(targetPos));
        CommandScheduler.getInstance().schedule(AutoCommandFactory.followPathCommand(trajectory, false));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
