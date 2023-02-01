package frc.robot.factories;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import frc.robot.subsystems.*;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandFactory {

    private static Swerve s_Swerve = Swerve.getInstance();
    private static Command lastCommand;
    private static Command selectedAuto;

    public static Command getAutoCommand(AutoType auto) {
        switch (auto) {
            case Straight:
                return selectedAuto = straight();
            case Right:
                return selectedAuto = forwardAndRightCommand();
            case Wait:
                return selectedAuto = pathWithWait();
            case Test:
                return selectedAuto = test();
        }
        return null;
    }

    public enum AutoType {
        Straight,
        Right,
        Wait,
        Test,
    }

    public static Command getSelectedAuto() {
        return selectedAuto;
    }

    public static void cancelLastCommand() {
        lastCommand.cancel();
    }

    public static Command followPathCommand(PathPlannerTrajectory path, boolean isFirstPath) {
        PIDController xController = new PIDController(0, 0, 0);
        PIDController yController = new PIDController(0, 0, 0);
        PIDController thetaController = new PIDController(0, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        lastCommand = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (isFirstPath) {
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
        return lastCommand;
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
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("path with wait event",
                new PathConstraints(3.5, 2));
        return new SequentialCommandGroup(
                followPathCommand(pathGroup.get(0), true),
                new WaitCommand(2),
                followPathCommand(pathGroup.get(1), false));
    }

    private static Command test() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("test",
                new PathConstraints(4, 3));
        return new SequentialCommandGroup(
                new WaitCommand(2),
                followPathCommand(pathGroup.get(0), true),
                new WaitCommand(2),
                followPathCommand(pathGroup.get(1), false));
    }
}
