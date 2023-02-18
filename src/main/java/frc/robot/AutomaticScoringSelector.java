package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;

public final class AutomaticScoringSelector {

    private static AutomaticScoringSelector instance;

    public static AutomaticScoringSelector getInstance(){
        if(instance == null){
            instance = new AutomaticScoringSelector();
        }
        return instance;
    }   

    public enum ScoringPositions {
        ALLPOSES();
        
        //botbot refers to the bottom grid of 9, and the bottom row in that grid. L1 refers to height level. Orientation is bird's eye, same as path planner
        ArrayList<ScoringPosition> coneStates; //it goes: [botbotL1, botbotL2, botbotL3, bottopL1... ...toptopL3]
        ArrayList<ScoringPosition> cubeStates; //it goes: [botmidL1, botmidL2, botmidL3, midmidL1... ...topmidL3]

        private ScoringPositions(){
            double x = 0.0; //should be the same for every default scoring position
            double y = 0.0; //only one that changes
            double yIncrem = 5.0; //how much each y varies by - currently unmeasured
            Rotation2d rot = new Rotation2d(0.0); // should be the same for ever default scoring position
            ArmStates[] armStatesCone = {ArmStates.L1CONE, ArmStates.L2CONE, ArmStates.L3CONE};
            ArmStates[] armStatesCube = {ArmStates.L1CUBE, ArmStates.L2CUBE, ArmStates.L3CUBE};
            ElevatorStates[] elevStatesCone = {ElevatorStates.L1CONE, ElevatorStates.L2CONE, ElevatorStates.L3CONE};
            ElevatorStates[] elevStatesCube = {ElevatorStates.L1CUBE, ElevatorStates.L2CUBE, ElevatorStates.L3CUBE};

            for(int i = 1; i <= 3; i++){
                for(int j = 1; j <= 3; j++){
                    y += yIncrem;
                    Pose2d tempPose = new Pose2d(x, y, rot);
                    for(int b = 1; b <= 3; b++){
                        if(j%2==0){
                            cubeStates.add(AutomaticScoringSelector.getInstance().new ScoringPosition(tempPose, armStatesCube[b - 1], elevStatesCube[b - 1]));
                        } else {
                            coneStates.add(AutomaticScoringSelector.getInstance().new ScoringPosition(tempPose, armStatesCone[b - 1], elevStatesCone[b - 1]));
                        }
                    }
                }
            }
        }
    }

    class ScoringPosition {

        Pose2d targetPos;
        ArmStates armState;
        ElevatorStates elevatorState;

        private ScoringPosition(Pose2d pose, ArmStates armState, ElevatorStates elevState){
            targetPos = pose;
            this.armState = armState;
            this.elevatorState = elevState;
        }
    }
}
