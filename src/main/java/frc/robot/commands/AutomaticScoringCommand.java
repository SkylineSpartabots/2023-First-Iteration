package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutomaticScoringSelector;
import frc.robot.ScoringPosition;
import frc.robot.subsystems.Limelight;

public class AutomaticScoringCommand extends CommandBase{

    private ScoringPosition scoringPosition;
    private AutomaticScoringSelector selector;
    private Limelight limelight;

    public AutomaticScoringCommand(){
        selector = AutomaticScoringSelector.getInstance();
        limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {
        scoringPosition = selector.getScoringPosition();
        boolean positionReset = false;
        while(!positionReset){
            positionReset = limelight.hasTarget();
        }
        new SmartResetOdometry().schedule();
        new SequentialCommandGroup(
            new OnTheFlyGeneration(scoringPosition.getTargetPos(), true),
            new SetArm(scoringPosition.getArmState()),
            new SetElevator(scoringPosition.getElevatorState())
            ).schedule();
    }

    private static double maxVel = 3; //4
    private static double maxAccel = 2; //3

    public static double getMaxAccel() {
        return maxAccel;
    }
    
    public static double getMaxVel() {
        return maxVel;
    }
}
