package frc.robot.factories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

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
        boolean atPosition;
        if(targetPose == null){
            atPosition = true;
        } else {
            // atPosition = s_Swerve.inScoringPosition(targetPose);
        }
        switch (type) {
        //     case Low:
        //         return new SequentialCommandGroup(new SetMechanism(MechanismState.L1CONE),
        //          new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED_CONE));
        //     case MidCone:
        //         return new SequentialCommandGroup(new SetMechanism(MechanismState.L2CONE),
        //          new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED_CONE));
        //     case HighCone:
        //         return new SequentialCommandGroup(new SetMechanism(MechanismState.L3CONE),
        //          new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED_CONE));
        //     case MidCube:
        //         return new SequentialCommandGroup(new SetMechanism(MechanismState.L2CUBE),
        //          new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED_CONE));
        //     case HighCube:
        //         return new SequentialCommandGroup(new SetMechanism(MechanismState.L2CONE),
        //          new WaitUntilCommand(s_Swerve.inScoringPosition(targetPose)), new SetIntake(IntakeStates.OFF_DEPLOYED_CONE));
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
