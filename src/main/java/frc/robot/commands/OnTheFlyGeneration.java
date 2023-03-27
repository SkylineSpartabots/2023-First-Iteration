/*
 OTF command that takes a target pose and uses robot
 current pose to generate a trajectroy between the two
*/

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.subsystems.Swerve;

public class OnTheFlyGeneration extends CommandBase {
    Pose2d currentPos;
    Pose2d targetPos;
    Swerve s_Swerve;
    boolean autoScoring;
    double heading;
    boolean done;

    public OnTheFlyGeneration(Pose2d targetPos) {
        s_Swerve = Swerve.getInstance();
        this.targetPos = targetPos;
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
        double x = targetPos.getX() - currentPos.getX();
        double y = targetPos.getY() - currentPos.getY();
        // heading to find angle between two points so that it drives straight 
        // between the two points and does not curve weird. This is a niche 
        // concept about the path planner library.
        heading = Math.atan2(y, x);
        double maxVel = 3;
        double maxAccel = 2;
        // generates trajectory with robot current psoe and target pose
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(maxVel, maxAccel),
                getPathPoint(currentPos),
                getPathPoint(targetPos));
        // start and end pose values for command, uncomment for debugging
        /* SmartDashboard.putNumber("OTF-start-x", currentPos.getX());
        SmartDashboard.putNumber("OTF-start-y", currentPos.getY());
        SmartDashboard.putNumber("OTF-end-x", targetPos.getX());
        SmartDashboard.putNumber("OTF-end-y", targetPos.getY()); */
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
        return done;
    }

}
