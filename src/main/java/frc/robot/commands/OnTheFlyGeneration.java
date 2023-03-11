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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutomaticScoringSelector;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.subsystems.Swerve;

public class OnTheFlyGeneration extends CommandBase {
    Pose2d currentPos;
    Pose2d targetPos;
    Swerve s_Swerve;
    boolean autoScoring;
    // AutomaticScoringSelector selector;
    double heading;
    boolean done;

    public OnTheFlyGeneration(Pose2d targetPos) {
        s_Swerve = Swerve.getInstance();
        // selector = AutomaticScoringSelector.getInstance();
        // this.autoScoring = autoScoring;
        this.targetPos = targetPos;
        // addRequirements(s_Swerve);
        done = false;
    }

    private PathPoint getPathPoint(Pose2d pose) {
        return new PathPoint(new Translation2d(pose.getX(), pose.getY()),
                Rotation2d.fromRadians(heading),
                pose.getRotation());
    }

    @Override
    public void initialize() {
        currentPos = s_Swerve.getPose();
        // if (autoScoring) {
        //     targetPos = selector.getSelectedPose();
        // }
        double x = targetPos.getX() - currentPos.getX();
        double y = targetPos.getY() - currentPos.getY();
        heading = Math.atan2(y, x);
        double maxVel = 3;
        double maxAccel = 2;
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                getPathPoint(currentPos),
                getPathPoint(targetPos));
        // SmartDashboard.putNumber("OTF-start-x", currentPos.getX());
        // SmartDashboard.putNumber("OTF-start-y", currentPos.getY());
        // SmartDashboard.putNumber("OTF-end-x", targetPos.getX());
        // SmartDashboard.putNumber("OTF-end-y", targetPos.getY());
        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                AutoCommandFactory.followPathCommand(trajectory),
                new InstantCommand(() -> {
                    done = true;
                })
            )
        );
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // SmartDashboard.putNumber("FUCK", counter);
        // return AutomaticScoringSelector.getInstance().inPosition();
        return done;
    }

}
