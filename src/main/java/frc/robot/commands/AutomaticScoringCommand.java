package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutomaticScoringSelector;
import frc.robot.ScoringPosition;
import frc.robot.subsystems.Limelight;

public class AutomaticScoringCommand extends CommandBase {

    private ScoringPosition scoringPosition;
    private AutomaticScoringSelector selector;
    private Limelight limelight;

    public AutomaticScoringCommand() {
        selector = AutomaticScoringSelector.getInstance();
        limelight = Limelight.getInstance();
    }

    @Override
    public void initialize() {
        // scoringPosition = selector.getScoringPosition();
        boolean positionReset = false;
        while (!positionReset) {
            positionReset = limelight.hasTarget();
        }
        CommandScheduler.getInstance().schedule(new SmartResetOdometry());
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new OnTheFlyGeneration(scoringPosition.getTargetPos(), true),

                new WaitUntilCommand(AutomaticScoringSelector.getInstance().inPosition)
                        .andThen(new ParallelCommandGroup(new SetArm(scoringPosition.getArmState()),
                                new SetElevator(scoringPosition.getElevatorState())))));

        // new SetArm(scoringPosition.getArmState()),
        // new SetElevator(scoringPosition.getElevatorState())));
    }

    private static double maxVel = 3; // 4
    private static double maxAccel = 2; // 3

    public static double getMaxAccel() {
        return maxAccel;
    }

    public static double getMaxVel() {
        return maxVel;
    }
}
