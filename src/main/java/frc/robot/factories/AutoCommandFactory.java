/*
 auto command factory that creates all the autos
*/

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetMechanism;
import frc.robot.commands.SmartResetOdometry;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import frc.robot.subsystems.Intake.IntakeStates;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class AutoCommandFactory {

        private static Swerve s_Swerve = Swerve.getInstance();
        private static Intake s_Intake = Intake.getInstance();
        private static Command lastCommand;
        private static Command selectedAuto;

        // used to return selected auto based on the sendable chooser on shuffle board
        public static Command getAutoCommand(AutoType auto) {
                switch (auto) {
                        case Test:
                                return selectedAuto = test();
                        case OneCone:
                                return selectedAuto = oneCone();
                        case OneConeBackBottom:
                                return selectedAuto = oneConeBackBottom();
                        case OneConeDockMiddle:
                                return selectedAuto = oneConeDockMiddle();
                        case OneConeDockMiddleCom:
                                return selectedAuto = oneConeDockMiddleCom();
                        case OneHalfConeDockTop:
                                return selectedAuto = oneHalfConeDockTop();
                        case TwoConeTop:
                                return selectedAuto = twoConeTop();
                        case TwoConeBottom:
                                return selectedAuto = twoConeBottom();
                        case TwoConeDockTop:
                                return selectedAuto = twoConeDockTop();
                        case TwoConeDockBottom:
                                return selectedAuto = twoConeDockBottom();
                        case ThreeConeTop:
                                return selectedAuto = threeConeTop();
                        case ThreeConeBottom:
                                return selectedAuto = threeConeBottom();
                        default:
                                break;
                }
                return null;
        }

        public enum AutoType {
                Test,
                OneCone,
                OneConeBackBottom,
                OneConeDockMiddle,
                OneConeDockMiddleCom,
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

        // follow path command to execute a given trajectory, uses path planner library to generate
        // a trjaectory based on the path which is inputted from the .path files
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
        }

        public static Command followPathCommandTeleOP(PathPlannerTrajectory path) {
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
                                                false, // TODO
                                                s_Swerve));
                return lastCommand;
        }

        // used to generate init and end poses for trajectories from path states, have
        // to provide rotation manually
        public static Pose2d getPoseFromState(PathPlannerState state, double angleRotation) {
                state = PathPlannerTrajectory.transformStateForAlliance(state, DriverStation.getAlliance());
                Pose2d pose = new Pose2d(state.poseMeters.getX(), state.poseMeters.getY(),
                                new Rotation2d(Math.toRadians(angleRotation)));
                return pose;
        }

        // moves swerve forward until certain conditions are met
        public static void forwardUntilCommand() {
                double driveSpeed = 0.2;
                while (!Intake.getInstance().hasCube() && s_Swerve.getPose().getX() < 7) { // careful watch for hasCube
                                                                                           // vs hasCone
                        s_Swerve.drive(
                                        new Translation2d(driveSpeed, 0).times(Constants.SwerveConstants.maxSpeed),
                                        0,
                                        false,
                                        false);
                }
                s_Swerve.drive(
                                new Translation2d(0, 0), 0, false, false);
        }

        // test auto for testing purposes
        private static Command test() {
                PathPlannerTrajectory path = PathPlanner.loadPath("test", new PathConstraints(2.0, 1.5));
                Pose2d initPose = getPoseFromState(path.getInitialState(), 180);
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(initPose)),
                                new ParallelCommandGroup(
                                                followPathCommand(path),
                                                new SetMechanism(MechanismState.GROUNDINTAKE)
                                                                .andThen(new WaitCommand(0.7))
                                                                .andThen(new SetIntake(IntakeStates.ON_CLOSED_CONE))));
        }

        // simple one cube and no movement
        private static Command oneCone() {
                return new SequentialCommandGroup(
                                new SetMechanism(MechanismState.L3CUBE),
                                new WaitCommand(1),
                                new SetIntake(IntakeStates.REV_OPEN_CUBE),
                                new WaitCommand(0.8),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE));
        }

        // one cone and then back out of community bottom
        private static Command oneConeBackBottom() {
                PathPlannerTrajectory path = PathPlanner.loadPath("1 cone back bottom", new PathConstraints(2.0, 1.5));
                Pose2d initPose = getPoseFromState(path.getInitialState(), 180);
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(initPose)),
                                new SetMechanism(MechanismState.L3CONE),
                                new WaitCommand(1),
                                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                                new WaitCommand(0.8),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                                new SetMechanism(MechanismState.ZERO),
                                followPathCommand(path));
        }

        // one cone and then dock middle
        private static Command oneConeDockMiddle() {
                PathPlannerTrajectory path = PathPlanner.loadPath("1 cone dock middle", new PathConstraints(2.0, 0.5));
                Pose2d initPose = getPoseFromState(path.getInitialState(), 180);
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(initPose)),
                                new SetMechanism(MechanismState.L3CONE),
                                new WaitCommand(1),
                                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                                new WaitCommand(0.8),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                                new SetMechanism(MechanismState.ZERO),
                                followPathCommand(path),
                                // new WaitCommand(0.3),
                                new AutoBalance());
        }

        // one cone and then drive out of community and then dock middle
        private static Command oneConeDockMiddleCom() {
                List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("1 cone dock middle com",
                                new PathConstraints(3.0, 1.5),
                                new PathConstraints(2.0, 0.8),
                                new PathConstraints(2.0, 0.8));
                Pose2d initPose = getPoseFromState(pathGroup.get(0).getInitialState(), 180);
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(initPose)),
                                new SetMechanism(MechanismState.L3CONE),
                                new WaitCommand(0.8),
                                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                                new WaitCommand(0.5),
                                new SetIntake(IntakeStates.OFF_CLOSED_CONE),
                                new SetMechanism(MechanismState.ZERO),
                                followPathCommand(pathGroup.get(0)),
                                followPathCommand(pathGroup.get(1)),
                                // new WaitCommand(0.3),
                                followPathCommand(pathGroup.get(2)),
                                new AutoBalance());
        }

        // one cone and then one cube top
        private static Command twoConeTop() {
                List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("2 cone top",
                                new PathConstraints(5.0, 1.0),
                                new PathConstraints(5.0, 2));
                Pose2d initPose = getPoseFromState(pathGroup.get(0).getInitialState(), 180);
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(initPose)),
                                new SetMechanism(MechanismState.L3CONE),
                                new WaitCommand(0.5),
                                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> s_Swerve.goalPoseParameters(
                                                getPoseFromState(pathGroup.get(0).getEndState(), 0), 3.2, 3.0, 180)),
                                new ParallelCommandGroup(
                                                followPathCommand(pathGroup.get(0))
                                                                .andThen(new InstantCommand(
                                                                                () -> s_Swerve.drive(
                                                                                                new Translation2d(0, 0),
                                                                                                0, false,
                                                                                                false))),
                                                new SetIntake(IntakeStates.ON_OPEN_CUBE),
                                                new SetMechanism(MechanismState.ZERO),
                                                new WaitUntilCommand(s_Swerve.inPosition).andThen(
                                                                new SetMechanism(MechanismState.GROUNDINTAKE))
                                                                ).andThen(new InstantCommand(() -> s_Swerve.drive(new Translation2d(0.3, 0).times(Constants.SwerveConstants.maxSpeed), 
                                                                0, false, false))),
                                new WaitUntilCommand(s_Intake.shouldStopOnAuto),
                                new InstantCommand(()-> s_Swerve.drive(new Translation2d(0, 0), 0, false, false)),
                                new WaitCommand(0.3),
                                new ParallelCommandGroup(
                                                followPathCommand(pathGroup.get(1)),
                                                new SetMechanism(MechanismState.ZERO)),
                                new SmartResetOdometry(), 
                                followPathCommand(pathGroup.get(2)),
                                new SetMechanism(MechanismState.L3CUBE).andThen(new WaitCommand(0.8)),
                                new SetIntake(IntakeStates.REV_OPEN_CUBE));
        }

        // paths were never written because we never got the chance to
        private static Command oneHalfConeDockTop() {
                return new WaitCommand(0);
        }

        private static Command twoConeBottom() {
                return new WaitCommand(0);
        }

        private static Command twoConeDockBottom() {
                return new WaitCommand(0);
        }

        private static Command twoConeDockTop() {
                return new WaitCommand(0);
        }

        // never used because we never got the chance but it essentially does what it says
        private static Command threeConeTop() {
                List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("3 cone top",
                                new PathConstraints(5.0, 2),
                                new PathConstraints(5.0, 2),
                                new PathConstraints(5.0, 2),
                                new PathConstraints(5.0, 2));
                Pose2d initPose = getPoseFromState(pathGroup.get(0).getInitialState(), 180);
                return new SequentialCommandGroup(
                                new InstantCommand(() -> s_Swerve.resetOdometry(initPose)),
                                new SetMechanism(MechanismState.L3CONE),
                                new WaitCommand(0.5),
                                new SetIntake(IntakeStates.OFF_OPEN_CONE),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> s_Swerve.goalPoseParameters(
                                                getPoseFromState(pathGroup.get(0).getEndState(), 0), 2.4, 2.4, 180)),
                                new ParallelCommandGroup(
                                                followPathCommand(pathGroup.get(0))
                                                                .andThen(new InstantCommand(
                                                                                () -> s_Swerve.drive(
                                                                                                new Translation2d(0, 0),
                                                                                                0, false,
                                                                                                false))),
                                                new SetIntake(IntakeStates.ON_OPEN_CUBE),
                                                new SetMechanism(MechanismState.ZERO),
                                                new WaitUntilCommand(s_Swerve.inPosition).andThen(
                                                                new SetMechanism(MechanismState.GROUNDINTAKE))),
                                new WaitUntilCommand(s_Intake.motorStopped),
                                new WaitCommand(0.3),
                                new ParallelCommandGroup(
                                                followPathCommand(pathGroup.get(1)),
                                                new SetMechanism(MechanismState.ZERO)),
                                new SetMechanism(MechanismState.L3CUBE).andThen(new WaitCommand(0.8)),
                                new SetIntake(IntakeStates.REV_OPEN_CUBE),
                                new WaitCommand(0.5),
                                new InstantCommand(() -> s_Swerve.goalPoseParameters(
                                                getPoseFromState(pathGroup.get(2).getEndState(), 0), 2.4, 2.4, 180)),
                                new ParallelCommandGroup(
                                                followPathCommand(pathGroup.get(2))
                                                                .andThen(new InstantCommand(
                                                                                () -> s_Swerve.drive(
                                                                                                new Translation2d(0, 0),
                                                                                                0, false,
                                                                                                false))),
                                                new SetIntake(IntakeStates.ON_OPEN_CUBE),
                                                new SetMechanism(MechanismState.ZERO),
                                                new WaitUntilCommand(s_Swerve.inPosition).andThen(
                                                                new SetMechanism(MechanismState.GROUNDINTAKE))),
                                new WaitUntilCommand(s_Intake.motorStopped),
                                new WaitCommand(0.3),
                                new ParallelCommandGroup(
                                                followPathCommand(pathGroup.get(3)),
                                                new SetMechanism(MechanismState.ZERO)),
                                new SetMechanism(MechanismState.L2CUBE).andThen(new WaitCommand(0.8)),
                                new SetIntake(IntakeStates.REV_OPEN_CUBE)
                                );
        }

        private static Command threeConeBottom() {
                return new WaitCommand(0);
        }
}