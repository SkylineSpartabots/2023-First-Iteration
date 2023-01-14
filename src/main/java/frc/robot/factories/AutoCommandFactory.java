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
    
    private static final Swerve s_Swerve = Swerve.getInstance();
    private static Command selectedAuto;

    public static Command getAutoCommand(String auto) { 
        if (auto == "straightAuto")
            return selectedAuto = straight();
        else if (auto == "rightAuto")
            return selectedAuto = forwardAndRightCommand();
        else if (auto == "waitAuto")
            return selectedAuto = pathWithWait();
        else if (auto == "twoGP")
            return selectedAuto = twoGPBottom();
        return null;
    } 
    
    public static Command getSelectedAuto() {
        return selectedAuto;
    }

    public static Command followPathCommand(PathPlannerTrajectory path, boolean isFirstPath) {
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

    private static Command straight() {
        PathPlannerTrajectory path = PathPlanner.loadPath("test", new PathConstraints(4, 3));
        return followPathCommand(path, true);
    }

    private static Command forwardAndRightCommand() {
        PathPlannerTrajectory path = PathPlanner.loadPath("forward and right", new PathConstraints(4, 3));
        return followPathCommand(path, true);
    }

    private static Command pathWithWait() {
        ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("path with wait event",
            new PathConstraints(3.5, 2));
        return new SequentialCommandGroup(
            followPathCommand(pathGroup.get(0), true), 
            new WaitCommand(2), 
            followPathCommand(pathGroup.get(1), false));
    }

    private static Command twoGPBottom() {
        ArrayList<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("two cone", new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            new WaitCommand(1),
            followPathCommand(pathGroup.get(0), true),
            new WaitCommand(1.5),
            followPathCommand(pathGroup.get(1), false)
        );
    }

}
