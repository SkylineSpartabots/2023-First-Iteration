package frc.robot.factories;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.ArrayList;
import com.pathplanner.lib.PathConstraints;
import frc.robot.subsystems.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandFactory {
    
    private final Swerve s_Swerve = Swerve.getInstance();
    private Command selectedAuto;

    public Command getAutoCommand(String auto) { 
        if (auto == "straightAuto")
            return selectedAuto = straight();
        else if (auto == "rightAuto")
            return selectedAuto = forwardAndRightCommand();
        else if (auto == "waitAuto")
            return selectedAuto = pathWithWait();
        return null;
    } 
    
    public Command getSelectedAuto() {
        return selectedAuto;
    }

    private Command followPathCommand(PathPlannerTrajectory path, boolean isFirstPath) {
        PIDController xController = new PIDController(0, 0, 0);
        PIDController yController = new PIDController(0, 0, 0);
        PIDController thetaController = new PIDController(0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Command command = new SequentialCommandGroup(
            new InstantCommand(() -> {
            if(isFirstPath){
                s_Swerve.resetOdometry(new Pose2d());
            }
            }),
            new PPSwerveControllerCommand(
                path, 
                s_Swerve.poseSupplier,
                xController, 
                yController, 
                thetaController, 
                s_Swerve.chassisConsumer,
                s_Swerve));
        return command;
    }

    private Command straight() {
        PathPlannerTrajectory path = PathPlanner.loadPath("test", new PathConstraints(4, 3));
        return followPathCommand(path, true);
    }

    private Command forwardAndRightCommand() {
        PathPlannerTrajectory path = PathPlanner.loadPath("forward and right", new PathConstraints(4, 3));
        return followPathCommand(path, true);
    }

    private Command pathWithWait() {
        ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("path with wait event",
            new PathConstraints(3.5, 2));
        return new SequentialCommandGroup(
            followPathCommand(pathGroup.get(0), true), 
            new WaitCommand(2), 
            followPathCommand(pathGroup.get(1), false));
    }

}
