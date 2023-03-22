package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutomaticScoringSelector;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.CompleteMechanism;
import frc.robot.subsystems.CompleteMechanism.MechanismState;

public class AutoTeleopScore extends CommandBase {
    Pose2d pose;
    MechanismState mechanismState;
    Swerve s_Swerve;
    Limelight s_Limelight;
    CompleteMechanism s_CompleteMechanism;
    AutomaticScoringSelector s_AutomaticScoringSelector;
    boolean finished;

    public AutoTeleopScore() {
        s_Swerve = Swerve.getInstance();
        s_Limelight = Limelight.getInstance();
        s_CompleteMechanism = CompleteMechanism.getInstance();
        s_AutomaticScoringSelector = AutomaticScoringSelector.getInstance();
        addRequirements(s_Swerve, s_CompleteMechanism);
    }

    @Override
    public void initialize() {
        finished = false;
        if (s_Limelight.hasTarget()) {
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(
                            new SmartResetOdometry(),
                            new OnTheFlyGeneration(s_AutomaticScoringSelector.getSelectedPose()),
                            new SetMechanism(s_AutomaticScoringSelector.getMechState()),
                            new InstantCommand(() -> finished = true)
                    ));
        } else {
            finished = true;
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (finished) {
            SmartDashboard.putBoolean("AutoTeleop", false);
        }
        return finished;
    }

}
