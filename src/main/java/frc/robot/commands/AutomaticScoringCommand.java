package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutomaticScoringSelector;
import frc.robot.ScoringPosition;

public class AutomaticScoringCommand extends CommandBase{

    private ScoringPosition scoringPosition;
    private AutomaticScoringSelector selector;
    public AutomaticScoringCommand(){
        selector = AutomaticScoringSelector.getInstance();
    }

    @Override
    public void initialize() {
        scoringPosition = selector.getScoringPosition();
        Command command = new SequentialCommandGroup(
            new OnTheFlyGeneration(scoringPosition.getTargetPos(), true),
            new SetArm(scoringPosition.getArmState()),
            new SetElevator(scoringPosition.getElevatorState())
            );
        command.schedule();
    }
    
}
