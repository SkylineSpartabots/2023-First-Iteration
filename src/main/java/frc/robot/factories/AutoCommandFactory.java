package frc.robot.factories;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import frc.robot.commands.AutoBalance;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetMechanism;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import frc.robot.subsystems.Intake.IntakeStates;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoCommandFactory {

    private static Swerve s_Swerve = Swerve.getInstance();
    private static Command lastCommand;
    private static Command selectedAuto;

    public static Command getAutoCommand(AutoType auto) {
        // return testAuto();
        return twoConeBottom();
        // switch (auto) {
        //     case Wait:
        //         return selectedAuto = pathWithWait();
        //     case OneConeDockMiddle:
        //         return selectedAuto = oneConeDockMiddle();
        //     case TwoConeBottom:
        //         return selectedAuto = twoConeBottom();
        //     case TwoConeDockBottom:
        //         return selectedAuto = twoConeDockBottom();
        //     case TwoConeDockTop:
        //         return selectedAuto = twoConeDockTop();
        //     case TwoConeTop:
        //         return selectedAuto = twoConeTop();
        //     case ThreeConeTop:
        //         return selectedAuto = threeConeTop();
        //     case ThreeConeBottom:
        //         return selectedAuto = threeConeBottom();
        // }
        // return null;
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

    public static Command followPathCommand(PathPlannerTrajectory path) {
        PIDController xController = new PIDController(5, 0, 0);
        PIDController yController = new PIDController(5, 0, 0);
        PIDController thetaController = new PIDController(2, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        lastCommand = new SequentialCommandGroup(
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

    private static Command testAuto() {
        PathPlannerTrajectory path = PathPlanner.loadPath("test", new PathConstraints(4, 3));
        return followPathCommand(path);

    }
 
    private static Command pathWithWait() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("path with wait event",
                new PathConstraints(4, 3));
        return new SequentialCommandGroup(
                followPathCommand(pathGroup.get(0)),
                new WaitCommand(2),
                followPathCommand(pathGroup.get(1)));
    }

    private static Command oneConeDockMiddle() {
        PathPlannerTrajectory path = PathPlanner.loadPath("1 cone dock middle", new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.86, 2.71, new Rotation2d(Math.toRadians(180))))),
            followPathCommand(path)
            // new AutoBalance()
        );
    }

    private static Command twoConeBottom() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone bottom",
        new PathConstraints(3.5, 1.0));
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(2.07, 1.04, new Rotation2d(Math.toRadians(180))))),
            new SetMechanism(MechanismState.MIDCUBE),
            new SetIntake(IntakeStates.REV_RETRACTED),
            new WaitCommand(0.5),
            new SetIntake(IntakeStates.OFF_RETRACTED),
            new ParallelCommandGroup(
                followPathCommand(pathGroup.get(0)),
                new SetMechanism(MechanismState.ZERO).andThen(new SetMechanism(MechanismState.CUBEINTAKE))
            ),
            new SetIntake(IntakeStates.ON_RETRACTED),
            new WaitCommand(1.0),
            new SetIntake(IntakeStates.OFF_RETRACTED),
            new ParallelCommandGroup(
                followPathCommand(pathGroup.get(1)),
                new SetMechanism(MechanismState.ZERO).andThen(new SetMechanism(MechanismState.LOWCUBE))
            ),
            new SetIntake(IntakeStates.REV_RETRACTED),
            new WaitCommand(0.8),
            new SetIntake(IntakeStates.OFF_RETRACTED)
        );
    }

    private static Command twoConeDockBottom() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone dock bottom",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.92, 1.05, new Rotation2d(Math.toRadians(180))))),
            // put cone
            followPathCommand(pathGroup.get(0)),
            // pick up cone
            followPathCommand(pathGroup.get(1)),
            // put cone
            followPathCommand(pathGroup.get(2)),
            new AutoBalance()
        );
    }

    private static Command twoConeDockTop() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone dock top",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.83, 4.42, new Rotation2d(Math.toRadians(180))))),
            // put cone
            followPathCommand(pathGroup.get(0) ),
            // pick up cone
            followPathCommand(pathGroup.get(1) ),
            // put cone
            followPathCommand(pathGroup.get(2) ),
            new AutoBalance()
        );
    }

    private static Command twoConeTop() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone top",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.83, 4.42, new Rotation2d(Math.toRadians(180))))),
            // put cone
            followPathCommand(pathGroup.get(0) ),
            // pick up cone
            followPathCommand(pathGroup.get(1) )
            // put cone

        );
    }
    private static Command threeConeTop() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 cone top",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.96, 4.42, new Rotation2d(Math.toRadians(180))))),
            // put cone
            followPathCommand(pathGroup.get(0) ),
            // pick up cone
            followPathCommand(pathGroup.get(1) ),
            // put cone
            followPathCommand(pathGroup.get(2) ),
            // pick up cone
            followPathCommand(pathGroup.get(3) )
            // put cone

        );
    }

    private static Command threeConeBottom() {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 cone bottom",
        new PathConstraints(4, 3));
        return new SequentialCommandGroup(
            new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(1.94, 1.07, new Rotation2d(Math.toRadians(180))))),
            // put cone
            followPathCommand(pathGroup.get(0) ),
            // pick up cone
            followPathCommand(pathGroup.get(1) ),
            // put cone
            followPathCommand(pathGroup.get(2) ),
            // pick up cone
            followPathCommand(pathGroup.get(3) )
            // put cone

        );
    }
 }
