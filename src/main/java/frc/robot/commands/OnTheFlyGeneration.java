package frc.robot.commands;


import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class OnTheFlyGeneration extends CommandBase {
    Pose2d currentPos;
    Pose2d targetPos;
    boolean swervePose;
    private Swerve s_Swerve;

    public OnTheFlyGeneration(Pose2d currentPos, Pose2d targetPos, boolean swervePose) {
        s_Swerve = Swerve.getInstance();
        this.targetPos = new Pose2d(targetPos.getX(), targetPos.getY(), targetPos.getRotation());
        this.currentPos = currentPos;
        this.swervePose = swervePose;
        addRequirements(s_Swerve);
    }
    
    public OnTheFlyGeneration(Pose2d currentPos, int targetID, boolean swervePose) {
        s_Swerve = Swerve.getInstance();
        this.currentPos = currentPos;
        this.targetPos = Constants.Limelight.gameAprilTags2d[targetID].plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
        this.swervePose = swervePose;
        addRequirements(s_Swerve);
    }
    
    public OnTheFlyGeneration(int targetID, boolean swervePose) {
        s_Swerve = Swerve.getInstance();
        this.targetPos = Constants.Limelight.gameAprilTags2d[targetID].plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()));
        this.swervePose = swervePose;
        addRequirements(s_Swerve);
    }

    private PathPoint getPathPoint(Pose2d pose) {
        return new PathPoint(new Translation2d(pose.getX(), pose.getY()),
                Rotation2d.fromDegrees(0),
                pose.getRotation());
    }

    @Override
    public void initialize() {
        if(swervePose) {
            currentPos = s_Swerve.getPose();
            // targetPos = new Pose2d(new Translation2d(0, 0), new Rotation2d());
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
