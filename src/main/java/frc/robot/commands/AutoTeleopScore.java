package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutomaticScoringSelector;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CompleteMechanism.MechanismState;

public class AutoTeleopScore extends CommandBase {
    Pose2d pose;
    MechanismState mechanismState;
    Intake s_Intake;

    public AutoTeleopScore() {
        s_Intake = Intake.getInstance();
        // Pose2d currentPose = AutomaticScoringSelector.
    }

    @Override
    public void initialize() {
        pose = AutomaticScoringSelector.getInstance().getSelectedPose();
        mechanismState = AutomaticScoringSelector.getInstance().getMechState(); 
    }

    @Override
    public void execute() {


        SmartDashboard.putBoolean("AS in pos", AutomaticScoringSelector.getInstance().inPosition());

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new OnTheFlyGeneration(new Pose2d(), true),
                        new SetMechanism(mechanismState)
                        // new WaitUntilCommand(AutomaticScoringSelector.getInstance().inPosition)
                        //         .andThen(new SetMechanism(mechanismState)),
        ));
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
