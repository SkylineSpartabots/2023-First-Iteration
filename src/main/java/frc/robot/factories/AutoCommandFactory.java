package frc.robot.factories;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import frc.robot.commands.AutoBalance;
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
            case Wait:
                return selectedAuto = pathWithWait();
            case OneConeDockMiddle:
                return selectedAuto = oneConeDockMiddle();
            case TwoConeBottom:
                return selectedAuto = twoConeBottom();
            case TwoConeDockBottom:
                return selectedAuto = twoConeDockBottom();
            case TwoConeDockTop:
                return selectedAuto = twoConeDockTop();
            case TwoConeTop:
                return selectedAuto = twoConeTop();
            case ThreeConeTop:
                return selectedAuto = threeConeTop();
            case ThreeConeBottom:
                return selectedAuto = threeConeBottom();
        }
        return null;
    }

    public enum AutoType {
        Wait,
        OneConeDockMiddle,
        TwoConeBottom,
        TwoConeDockBottom,
        TwoConeDockTop,
        TwoConeTop,
        ThreeConeTop,
        ThreeConeBottom,
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

    private static Command pathWithWait() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("path with wait event",
                new PathConstraints(4, 3));
        return new SequentialCommandGroup(
                followPathCommand(pathGroup.get(0), true),
                new WaitCommand(2),
                followPathCommand(pathGroup.get(1), false));
    }

    private static Command oneConeDockMiddle() {
        PathPlannerTrajectory path = PathPlanner.loadPath("1 cone dock middle", new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            // put cone command
            followPathCommand(path, true),
            new AutoBalance()
        );
    }

    private static Command twoConeBottom() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone bottom",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            // put cone command
            followPathCommand(pathGroup.get(0), true),
            // pick cone command
            followPathCommand(pathGroup.get(1), false)
            // put cone command
        );
    }

    private static Command twoConeDockBottom() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone dock bottom",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            // put cone
            followPathCommand(pathGroup.get(0), true),
            // pick up cone
            followPathCommand(pathGroup.get(1), false),
            // put cone
            followPathCommand(pathGroup.get(2), false),
            new AutoBalance()
        );
    }

    private static Command twoConeDockTop() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone dock top",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            // put cone
            followPathCommand(pathGroup.get(0), true),
            // pick up cone
            followPathCommand(pathGroup.get(1), false),
            // put cone
            followPathCommand(pathGroup.get(2), false),
            new AutoBalance()
        );
    }

    private static Command twoConeTop() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone top",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            // put cone
            followPathCommand(pathGroup.get(0), true),
            // pick up cone
            followPathCommand(pathGroup.get(1), false)
            // put cone

        );
    }
    private static Command threeConeTop() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 cone top",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            // put cone
            followPathCommand(pathGroup.get(0), true),
            // pick up cone
            followPathCommand(pathGroup.get(1), false),
            // put cone
            followPathCommand(pathGroup.get(2), false),
            // pick up cone
            followPathCommand(pathGroup.get(3), false)
            // put cone

        );
    }

    private static Command threeConeBottom() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 cone bottom",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            // put cone
            followPathCommand(pathGroup.get(0), true),
            // pick up cone
            followPathCommand(pathGroup.get(1), false),
            // put cone
            followPathCommand(pathGroup.get(2), false),
            // pick up cone
            followPathCommand(pathGroup.get(3), false)
            // put cone

        );
    }
 }
