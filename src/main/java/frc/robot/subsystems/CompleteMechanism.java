package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;


public class CompleteMechanism extends SubsystemBase {
    
    private static CompleteMechanism combScoring;
    private MechanismState currentState = MechanismState.REALZERO;
    Arm s_Arm = Arm.getInstance();
    Elevator s_Elevator = Elevator.getInstance();

    public static CompleteMechanism getInstance(){
        if(combScoring == null){
            combScoring = new CompleteMechanism();
        }
        return combScoring;
    }

    public enum MechanismState{
        REALZERO(ElevatorStates.REALZERO, ArmStates.ZERO),
        ZERO(ElevatorStates.ZERO, ArmStates.ZERO),

        L1CONE(ElevatorStates.L1CONE, ArmStates.L1CONE),
        L2CONE(ElevatorStates.L2CONE, ArmStates.L2CONE),
        L3CONE(ElevatorStates.L3CONE, ArmStates.L3CONE),

        L1CUBE(ElevatorStates.L1CUBE, ArmStates.L1CUBE),
        L2CUBE(ElevatorStates.L2CUBE, ArmStates.L2CUBE),
        L3CUBE(ElevatorStates.L3CUBE, ArmStates.L3CUBE),

        // L1LAYEDCONE(ElevatorStates.L1LAYEDCONE, ArmStates.L1LAYEDCONE),
        // L2LAYEDCONE(ElevatorStates.L2LAYEDCONE, ArmStates.L2LAYEDCONE),
        // L3LAYEDCONE(ElevatorStates.L3LAYEDCONE, ArmStates.L3LAYEDCONE),
        
        CUBEINTAKE(ElevatorStates.CUBEINTAKE, ArmStates.CUBEINTAKE),
        CONEINTAKE(ElevatorStates.CONEINTAKE, ArmStates.CONEINTAKE),
        LAYEDCONE(ElevatorStates.LAYEDCONE, ArmStates.LAYEDCONE),
        SUBSTATION(ElevatorStates.SUBSTATION, ArmStates.SUBSTATION),
        DOUBLESUBSTATION(ElevatorStates.DOUBLESUBSTATION, ArmStates.DOUBLESUBSTATION);

        public ElevatorStates elevState;
        public ArmStates armState;

        private MechanismState(ElevatorStates elevState, ArmStates armState){
            this.elevState = elevState; this.armState = armState;
        }
    }


    public MechanismState getState(){
        // if(currentState == null){
        //     currentState = MechanismState.ZERO;
        // }
        return currentState;
    }

    public void setState(MechanismState state){
        currentState = state;
        // SetMechanism command = new SetMechanism(state);
        // command.schedule();
    }

    public boolean inState() {
        return ((Math.abs(s_Elevator.getCANCoderSetpoint() - s_Elevator.getCANCoderPosition()) < 45) && 
            (Math.abs(s_Arm.getCANCoderSetpoint() - s_Arm.getCANCoderPosition()) < 15));
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("mech state", inState());
    }
}
