package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.factories.AutoCommandFactory;

public class OnTheFlyGeneration extends CommandBase {
    Pose2d currentPos;
    Pose2d targetPos;

    public OnTheFlyGeneration(Pose2d currentPos, Pose2d targetPos) {
        this.targetPos = targetPos;
        this.currentPos = currentPos;
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
