package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutomaticScoringSelector;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CompleteMechanism;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class AutoTeleopScore extends CommandBase {
    Pose2d pose;
    MechanismState mechanismState;
    Intake s_Intake;
    Limelight s_Limelight;
    CompleteMechanism s_CompleteMechanism;
    AutomaticScoringSelector s_AutomaticScoringSelector;
    boolean finished;

    public AutoTeleopScore() {
        s_Intake = Intake.getInstance();
        s_Limelight = Limelight.getInstance();
        s_CompleteMechanism = CompleteMechanism.getInstance();
        s_AutomaticScoringSelector = AutomaticScoringSelector.getInstance();
        // addRequirements(Swerve.getInstance());
    }

    @Override
    public void initialize() {
        finished = false;
        // if (s_Limelight.hasTarget()) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new SmartResetOdometry(),
                            new OnTheFlyGeneration(new Pose2d(), true),
                            // new WaitUntilCommand(s_AutomaticScoringSelector.inPosition).andThen(new SetMechanism(MechanismState.L2CONE)),
                            new SetMechanism(MechanismState.L2CONE),
                            // new SetElevator(ElevatorStates.L2CONE),
                            // new SetMechanism(s_AutomaticScoringSelector.getMechState()),
                            new InstantCommand(() -> finished = true)
                    // new WaitUntilCommand(s_AutomaticScoringSelector.inPosition)
                    // .andThen(new SetMechanism(mechanismState)),
                    ));
        // }
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
