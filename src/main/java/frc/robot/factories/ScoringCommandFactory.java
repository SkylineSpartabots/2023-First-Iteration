package frc.robot.factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetMechanism;
import frc.robot.subsystems.CompleteMechanism;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import frc.robot.subsystems.Intake.IntakeStates;

public class ScoringCommandFactory {
    //a bit redundant right now but if we ever get to the point where we want variable reach for each type of scoring this would be useful
    //ie. if we start scoring from places not directly in front of the slot we want to score

    Swerve s_Swerve = Swerve.getInstance();
    
    private static ScoringCommandFactory scoringFact;

    public static ScoringCommandFactory getInstance(){
        if(scoringFact == null){
            scoringFact = new ScoringCommandFactory();
        }
        return scoringFact;
    }
    
    public Command getScoringCommand(ScoreCommandType type, Pose2d targetPose){
        //change the pose2Ds
        switch (type) {
            case Low:
                return new SequentialCommandGroup(new SetMechanism(MechanismState.LOWCONE),
                 new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED));
            case MidCone:
                return new SequentialCommandGroup(new SetMechanism(MechanismState.MIDCONE),
                 new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED));
            case HighCone:
                return new SequentialCommandGroup(new SetMechanism(MechanismState.HIGHCONE),
                 new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED));
            case MidCube:
                return new SequentialCommandGroup(new SetMechanism(MechanismState.MIDCUBE),
                 new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED));
            case HighCube:
                return new SequentialCommandGroup(new SetMechanism(MechanismState.MIDCONE),
                 new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED));
            default:
                return null;
        }
    }

    public enum ScoreCommandType {
        Low,
        MidCone,
        HighCone,
        MidCube,
        HighCube,
    }
}
