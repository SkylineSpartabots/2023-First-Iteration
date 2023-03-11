package frc.robot.factories;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import java.sql.DriverAction;
import java.util.List;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.OnTheFlyGeneration;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetMechanism;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import frc.robot.subsystems.Intake.IntakeStates;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoCommandFactory {

        private static Swerve s_Swerve = Swerve.getInstance();
        private static Command lastCommand;
        private static Command selectedAuto;

        public static Command getAutoCommand(AutoType auto) {
                switch (auto) {
                        case OneCone:
                                return selectedAuto = oneCone();
                        case OneConeBack:
                                return selectedAuto = oneConeBack();
                        case OneConeDockMiddle:
                                return selectedAuto = oneConeDockMiddle();
                        // case OneHalfConeDockTop:
                        //         return selectedAuto = oneHalfConeDockTop();
                        // case TwoConeBottom:
                        //         return selectedAuto = twoConeBottom();
                        // case TwoConeDockBottom:
                        //         // return selectedAuto = twoConeDockBottom();
                        // case TwoConeDockTop:
                        //         // return selectedAuto = twoConeDockTop();
                        case TwoConeTop:
                                return selectedAuto = twoConeTop();
                        // case ThreeConeTop:
                        //         // return selectedAuto = threeConeTop();
                        // case ThreeConeBottom:
                                // return selectedAuto = threeConeBottom();
                }
                return null;
        }

        public enum AutoType {
                OneCone,
                OneConeBack,
                OneConeDockMiddle,
                OneHalfConeDockTop,
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
                                                true,
                                                s_Swerve));
                return lastCommand;

                // SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(null, null, null, null,
                // null, null, null)
        }

        public static void forwardUntilCommand(){
                double driveSpeed = 0.2;
                while(!Intake.getInstance().hasCone() && s_Swerve.getPose().getX() < 7.6){
                        s_Swerve.drive(
                                new Translation2d(driveSpeed, 0).times(Constants.SwerveConstants.maxSpeed), 
                                0, 
                                true, 
                                true);
                }
                s_Swerve.drive(
                        new Translation2d(0,0), 0, true, true);
        }

        private static Command oneCone() {
                return new SequentialCommandGroup(
                        new SetMechanism(MechanismState.L2CONE),
                        new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                        new WaitCommand(1),
                        new SetIntake(IntakeStates.OFF_OPEN_CONE));
        }

        private static Command oneConeBack() {
                PathPlannerTrajectory path = PathPlanner.loadPath("1 cone back", new PathConstraints(2.0, 1.5));
                PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(path.getInitialState(),
                                DriverStation.getAlliance());
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(initState.poseMeters.getX(),
                                                initState.poseMeters.getY(), new Rotation2d(Math.toRadians(180))))),
                                new SetMechanism(MechanismState.L2CONE),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                                new WaitCommand(1),
                                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                                new WaitCommand(0.8),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                                new SetMechanism(MechanismState.ZERO),
                                followPathCommand(path));
        }

        private static Command oneConeDockMiddle() {
                PathPlannerTrajectory path = PathPlanner.loadPath("1 cone dock middle", new PathConstraints(2.0, 1.5));
                PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(path.getInitialState(),
                                DriverStation.getAlliance());
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(initState.poseMeters.getX(),
                                                initState.poseMeters.getY(), new Rotation2d(Math.toRadians(180))))),
                                new SetMechanism(MechanismState.L2CONE),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                                new WaitCommand(1),
                                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                                new WaitCommand(0.8),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                                new SetMechanism(MechanismState.ZERO),
                                followPathCommand(path),
                                new AutoBalance()
                );
        }

        private static Command twoConeTop(){//slow moving + stops smartly when current spikes
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone top",
                        new PathConstraints(2, 1.5));
        PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(
                        pathGroup.get(0).getInitialState(),
                        DriverStation.getAlliance());
        return new SequentialCommandGroup(
                new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(initState.poseMeters.getX(),
                                initState.poseMeters.getY(), new Rotation2d(Math.toRadians(180))))),
                new SetMechanism(MechanismState.L3CONE),
                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                new WaitCommand(1),
                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                new SetMechanism(MechanismState.ZERO),
                new ParallelCommandGroup(
                        followPathCommand(pathGroup.get(0)).alongWith(
                                        new SetIntake(IntakeStates.ON_OPEN_CUBE)),
                        new ParallelDeadlineGroup(new WaitCommand(1.1),
                                        new SetMechanism(MechanismState.ZERO))
                                        .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
                new InstantCommand(() -> forwardUntilCommand()),
                new SetIntake(IntakeStates.OFF_OPEN_CUBE),
                new ParallelCommandGroup(
                        new OnTheFlyGeneration(pathGroup.get(1).getEndState().poseMeters),
                        new SetMechanism(MechanismState.ZERO).andThen(new WaitCommand(0.8)).andThen(new SetMechanism(MechanismState.L3CUBE))),
                new SetIntake(IntakeStates.REV_OPEN_CUBE)
            );
        }

        // private static Command oneHalfConeDockTop() {
        //         List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("1.5 cone dock top",
        //                         new PathConstraints(3.5, 2.0));
        //         PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(
        //                         pathGroup.get(0).getInitialState(),
        //                         DriverStation.getAlliance());
        //         return new SequentialCommandGroup(
        //                         new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(initState.poseMeters.getX(),
        //                                         initState.poseMeters.getY(), new Rotation2d(Math.toRadians(180))))),
        //                         new SetMechanism(MechanismState.L3CONE),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CONE),
        //                         new WaitCommand(1),
        //                         new SetIntake(IntakeStates.REV_DEPLOYED_CONE),
        //                         new WaitCommand(1.0),
        //                         new SetIntake(IntakeStates.OFF_RETRACTED_CONE),
        //                         new ParallelCommandGroup(
        //                                         followPathCommand(pathGroup.get(0)).alongWith(
        //                                                         new SetIntake(IntakeStates.ON_DEPLOYED_CUBE)),
        //                                         new ParallelDeadlineGroup(new WaitCommand(1.25),
        //                                                         new SetMechanism(MechanismState.ZERO))
        //                                                         .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
        //                         new WaitCommand(0.75),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CUBE),
        //                         new ParallelCommandGroup(
        //                                         followPathCommand(pathGroup.get(1)),
        //                                         new SetMechanism(MechanismState.ZERO)),
        //                         new AutoBalance());
        // }

        // private static Command twoConeBottom() {
        //         List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone bottom",
        //                         new PathConstraints(3.5, 2.0));
        //         PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(
        //                         pathGroup.get(0).getInitialState(),
        //                         DriverStation.getAlliance());
        //         return new SequentialCommandGroup(
        //                         new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(initState.poseMeters.getX(),
        //                                         initState.poseMeters.getY(), new Rotation2d(Math.toRadians(180))))),
        //                         new SetMechanism(MechanismState.L3CONE),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CONE),
        //                         new WaitCommand(1),
        //                         new SetIntake(IntakeStates.REV_DEPLOYED_CONE),
        //                         new WaitCommand(0.8),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CONE),
        //                         new ParallelCommandGroup(
        //                                         followPathCommand(pathGroup.get(0)).alongWith(
        //                                                         new SetIntake(IntakeStates.ON_DEPLOYED_CUBE)),
        //                                         new ParallelDeadlineGroup(new WaitCommand(1.1),
        //                                                         new SetMechanism(MechanismState.ZERO))
        //                                                         .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
        //                         new WaitCommand(0.75),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CUBE),
        //                         new ParallelCommandGroup(
        //                                         followPathCommand(pathGroup.get(1)),
        //                                         new SetMechanism(MechanismState.ZERO)
        //                                                         .andThen(new WaitCommand(0.8))
        //                                                         .andThen(new SetMechanism(MechanismState.L3CUBE))),
        //                         new SetIntake(IntakeStates.REV_DEPLOYED_CUBE),
        //                         new WaitCommand(0.8),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CONE));
        // }

        // // private static Command twoConeDockBottom() {
        // // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone
        // // dock bottom",
        // // new PathConstraints(3.5, 1.0));
        // // PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(
        // // pathGroup.get(0).getInitialState(),
        // // DriverStation.getAlliance());

        // // return new SequentialCommandGroup(
        // // new InstantCommand(() -> s_Swerve.resetOdometry(initState.poseMeters)),
        // // new SetMechanism(MechanismState.L2CONE)
        // // .andThen(new SetIntake(IntakeStates.OFF_DEPLOYED_CONE)),
        // // new SetIntake(IntakeStates.REV_DEPLOYED_CONE),
        // // new WaitCommand(0.5),
        // // new SetIntake(IntakeStates.OFF_RETRACTED_CONE),
        // // new ParallelCommandGroup(
        // // followPathCommand(pathGroup.get(0)),
        // // new SetMechanism(MechanismState.ZERO)
        // // .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
        // // new SetIntake(IntakeStates.ON_DEPLOYED_CUBE),
        // // new WaitCommand(1.0),
        // // new SetIntake(IntakeStates.OFF_RETRACTED_CUBE),
        // // new ParallelCommandGroup(
        // // followPathCommand(pathGroup.get(1)),
        // // new SetMechanism(MechanismState.ZERO)
        // // .andThen(new SetMechanism(MechanismState.L2CUBE))
        // // .andThen(new SetIntake(
        // // IntakeStates.OFF_DEPLOYED_CUBE))),
        // // new SetIntake(IntakeStates.REV_DEPLOYED_CUBE),
        // // new WaitCommand(0.8),
        // // new SetIntake(IntakeStates.OFF_RETRACTED_CONE),
        // // new ParallelCommandGroup(
        // // followPathCommand(pathGroup.get(2)),
        // // new SetMechanism(MechanismState.ZERO))
        // // // new AutoBalance()
        // // );
        // // }

        // private static Command twoConeTop() {
        //         List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone top",
        //                         new PathConstraints(3.5, 2.0));
        //         PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(
        //                         pathGroup.get(0).getInitialState(),
        //                         DriverStation.getAlliance());
        //         return new SequentialCommandGroup(
        //                         new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(initState.poseMeters.getX(),
        //                                         initState.poseMeters.getY(), new Rotation2d(Math.toRadians(180))))),
        //                         new SetMechanism(MechanismState.L3CONE),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CONE),
        //                         new WaitCommand(1),
        //                         new SetIntake(IntakeStates.REV_DEPLOYED_CONE),
        //                         new WaitCommand(0.8),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CONE),
        //                         new ParallelCommandGroup(
        //                                         followPathCommand(pathGroup.get(0)).alongWith(
        //                                                         new SetIntake(IntakeStates.ON_DEPLOYED_CUBE)),
        //                                         new ParallelDeadlineGroup(new WaitCommand(1.1),
        //                                                         new SetMechanism(MechanismState.ZERO))
        //                                                         .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
        //                         new WaitCommand(0.75),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CUBE),
        //                         new ParallelCommandGroup(
        //                                         followPathCommand(pathGroup.get(1)),
        //                                         new SetMechanism(MechanismState.ZERO)
        //                                                         .andThen(new WaitCommand(0.8))
        //                                                         .andThen(new SetMechanism(MechanismState.L3CUBE))),
        //                         new SetIntake(IntakeStates.REV_DEPLOYED_CUBE),
        //                         new WaitCommand(0.8),
        //                         new SetIntake(IntakeStates.OFF_DEPLOYED_CONE));
        // }

        // private static Command twoConeDockTop() {
        // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone
        // dock top",
        // new PathConstraints(3.5, 1.0));
        // PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(
        // pathGroup.get(0).getInitialState(),
        // DriverStation.getAlliance());

        // return new SequentialCommandGroup(
        // new InstantCommand(() -> s_Swerve.resetOdometry(new
        // Pose2d(initState.poseMeters.getX(),
        // initState.poseMeters.getY(), new Rotation2d(Math.toRadians(180))))),
        // new SetMechanism(MechanismState.L3CONE),
        // new SetIntake(IntakeStates.OFF_DEPLOYED_CONE),
        // new WaitCommand(1),
        // new SetIntake(IntakeStates.REV_DEPLOYED_CONE),
        // new WaitCommand(0.5),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CONE),
        // new ParallelCommandGroup(
        // followPathCommand(pathGroup.get(0)).alongWith(
        // new SetIntake(IntakeStates.ON_DEPLOYED_CUBE)),
        // new ParallelDeadlineGroup(new WaitCommand(1.25),
        // new SetMechanism(MechanismState.ZERO))
        // .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
        // new WaitCommand(0.75),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CUBE),
        // new ParallelCommandGroup(
        // followPathCommand(pathGroup.get(1)),
        // new SetMechanism(MechanismState.ZERO)
        // .alongWith(new SetIntake(IntakeStates.ON_DEPLOYED_CUBE))
        // .alongWith(new WaitCommand(0.8).andThen(new SetIntake(
        // IntakeStates.OFF_RETRACTED_CUBE)))
        // .andThen(new WaitCommand(0.8))
        // .andThen(new SetMechanism(MechanismState.L3CUBE))),
        // new SetIntake(IntakeStates.REV_DEPLOYED_CUBE),
        // new WaitCommand(0.75),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CONE),
        // new ParallelCommandGroup(
        // followPathCommand(pathGroup.get(2)),
        // new SetMechanism(MechanismState.ZERO))
        // // new AutoBalance()
        // );
        // }

        // private static Command threeConeTop() {
        // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 cone
        // top",
        // new PathConstraints(4, 3));
        // return new SequentialCommandGroup(
        // new InstantCommand(
        // () -> s_Swerve.resetOdometry(new Pose2d(1.96, 4.42,
        // new Rotation2d(Math.toRadians(180))))),
        // // put cone
        // followPathCommand(pathGroup.get(0)),
        // // pick up cone
        // followPathCommand(pathGroup.get(1)),
        // // put cone
        // followPathCommand(pathGroup.get(2)),
        // // pick up cone
        // followPathCommand(pathGroup.get(3))
        // // put cone

        // );
        // }

        // private static Command threeConeBottom() {
        // List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 cone
        // bottom",
        // new PathConstraints(3.5, 1.0));
        // PathPlannerState initState = PathPlannerTrajectory.transformStateForAlliance(
        // pathGroup.get(0).getInitialState(),
        // DriverStation.getAlliance());

        // return new SequentialCommandGroup(
        // new InstantCommand(() -> s_Swerve.resetOdometry(initState.poseMeters)),
        // new SetMechanism(MechanismState.L2CONE)
        // .andThen(new SetIntake(IntakeStates.OFF_DEPLOYED_CONE)),
        // new SetIntake(IntakeStates.REV_DEPLOYED_CONE),
        // new WaitCommand(0.5),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CONE),
        // new ParallelCommandGroup(
        // followPathCommand(pathGroup.get(0)),
        // new SetMechanism(MechanismState.ZERO)
        // .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
        // new SetIntake(IntakeStates.ON_DEPLOYED_CUBE),
        // new WaitCommand(1.0),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CUBE),
        // new ParallelCommandGroup(
        // followPathCommand(pathGroup.get(1)),
        // new SetMechanism(MechanismState.ZERO)
        // .andThen(new SetMechanism(MechanismState.L2CUBE))
        // .andThen(new SetIntake(
        // IntakeStates.OFF_DEPLOYED_CUBE))),
        // new SetIntake(IntakeStates.REV_DEPLOYED_CUBE),
        // new WaitCommand(0.8),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CONE),
        // // third piece - test
        // new ParallelCommandGroup(
        // followPathCommand(pathGroup.get(2)),
        // new SetMechanism(MechanismState.ZERO)
        // .andThen(new SetMechanism(MechanismState.CUBEINTAKE))),
        // new SetIntake(IntakeStates.ON_DEPLOYED_CUBE),
        // new WaitCommand(1.0),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CUBE),
        // new ParallelCommandGroup(
        // followPathCommand(pathGroup.get(3)),
        // new SetMechanism(MechanismState.ZERO)
        // .andThen(new SetMechanism(MechanismState.ZERO))
        // .andThen(new SetIntake(
        // IntakeStates.OFF_DEPLOYED_CUBE))),
        // new SetIntake(IntakeStates.REV_DEPLOYED_CUBE),
        // new WaitCommand(0.8),
        // new SetIntake(IntakeStates.OFF_RETRACTED_CONE));
        // }
}
