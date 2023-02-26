package frc.robot.commands;



import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutomaticScoringSelector;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.subsystems.Swerve;

public class OnTheFlyGeneration extends CommandBase {
    Pose2d currentPos;
    Pose2d targetPos;
    Swerve s_Swerve;
    boolean autoScoring;
    AutomaticScoringSelector selector;

    public OnTheFlyGeneration(Pose2d targetPos, boolean autoScoring) {
        s_Swerve = Swerve.getInstance();
        selector = AutomaticScoringSelector.getInstance();
        this.autoScoring = autoScoring;
        this.targetPos = targetPos;
        addRequirements(s_Swerve);
    }
    
    private PathPoint getPathPoint(Pose2d pose) {
        return new PathPoint(new Translation2d(pose.getX(), pose.getY()),
                Rotation2d.fromDegrees(0),
                pose.getRotation());
    }

    @Override
    public void initialize() {
        currentPos = s_Swerve.getPose();
        if(autoScoring) {
            targetPos = selector.getSelectedPose();
        }
    }

    @Override
    public void execute() {
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(4, 3),
                getPathPoint(currentPos),
                getPathPoint(targetPos));
        SmartDashboard.putNumber("OTF-start-x", currentPos.getX());
        SmartDashboard.putNumber("OTF-start-y", currentPos.getY());
        SmartDashboard.putNumber("OTF-end-x", targetPos.getX());
        SmartDashboard.putNumber("OTF-end-y", targetPos.getY());
        CommandScheduler.getInstance().schedule(AutoCommandFactory.followPathCommand(trajectory));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
