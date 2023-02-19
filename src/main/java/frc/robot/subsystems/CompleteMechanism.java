package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetMechanism;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;


public class CompleteMechanism extends SubsystemBase{
    
    private static CompleteMechanism combScoring;
    private MechanismState currentState;

    public static CompleteMechanism getInstance(){
        if(combScoring == null){
            combScoring = new CompleteMechanism();
        }
        return combScoring;
    }

    public enum MechanismState{
        LOWCONE(ElevatorStates.L1CONE, ArmStates.L1CONE),
        MIDCONE(ElevatorStates.L2CONE, ArmStates.L2CONE),
        UPPERCONE(ElevatorStates.L3CONE, ArmStates.L3CONE),
        CUBEINTAKE(ElevatorStates.GROUNDCUBE, ArmStates.GROUNDCUBE),
        CONEINTAKE(ElevatorStates.GROUNDCONE, ArmStates.GROUNDCONE),
        LOWCUBE(ElevatorStates.L1CUBE, ArmStates.L1CUBE),
        MIDCUBE(ElevatorStates.L2CUBE, ArmStates.L2CUBE),
        HIGHCUBE(ElevatorStates.L3CUBE, ArmStates.L3CUBE),
        ZERO(ElevatorStates.ZERO, ArmStates.ZERO);

        public ElevatorStates elevState;
        public ArmStates armState;

        private MechanismState(ElevatorStates elevState, ArmStates armState){
            this.elevState = elevState; this.armState = armState;
        }
    }

    public MechanismState getState(){
        if(currentState == null){
            currentState = MechanismState.ZERO;
        }
        return currentState;
    }

    public void setState(MechanismState state){
        currentState = state;
        // SetMechanism command = new SetMechanism(state);
        // command.schedule();
    }
}
