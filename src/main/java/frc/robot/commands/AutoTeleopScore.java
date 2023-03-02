package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutomaticScoringSelector;
import frc.robot.subsystems.CompleteMechanism.MechanismState;

public class AutoTeleopScore extends CommandBase {
    Pose2d pose;
    int row;
    int column;
    boolean cube;
    MechanismState mechState;

    public AutoTeleopScore() {
        // Pose2d currentPose = AutomaticScoringSelector.
    }

    @Override
    public void initialize() {
        pose = AutomaticScoringSelector.getInstance().getSelectedPose();
        row = AutomaticScoringSelector.getInstance().currRow;
        column = AutomaticScoringSelector.getInstance().currColumn;
    
        column %= 3;
        if(column == 2) {cube = true;}
        else {cube = false;}
    }

    @Override
    public void execute() {
        if(row == 0) {
            if(cube) {
                mechState = MechanismState.LOWCUBE;
            } else {
                mechState = MechanismState.LOWCONE;
            }
        } else if(row == 1) {
            if(cube) {
                mechState = MechanismState.MIDCUBE;
            } else {
                mechState = MechanismState.MIDCONE;
            }
        } else if(row == 2) {
            if(cube) {
                mechState = MechanismState.HIGHCUBE;
            } else {
                mechState = MechanismState.HIGHCONE;
            }
        }

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                new OnTheFlyGeneration(new Pose2d(), true),
                new WaitUntilCommand(AutomaticScoringSelector.getInstance().inPosition).andThen(new SetMechanism(mechState))
            )
        );
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
