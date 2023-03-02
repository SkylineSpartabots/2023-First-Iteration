package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Elevator.ElevatorStates;


public class CompleteMechanism extends SubsystemBase {
    
    private static CompleteMechanism combScoring;
    private MechanismState currentState;
    Arm s_Arm = Arm.getInstance();
    Elevator s_Elevator = Elevator.getInstance();

    public static CompleteMechanism getInstance(){
        if(combScoring == null){
            combScoring = new CompleteMechanism();
        }
        return combScoring;
    }

    public enum MechanismState{
        LOWCONE(ElevatorStates.L1CONE, ArmStates.L1CONE),
        MIDCONE(ElevatorStates.L2CONE, ArmStates.L2CONE),
        HIGHCONE(ElevatorStates.L3CONE, ArmStates.L3CONE),
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

    public boolean inState() {
        return ((Math.abs(s_Elevator.getCANCoderSetpoint() - s_Elevator.getCANCoderPosition()) < 45) && 
            (Math.abs(s_Arm.getCANCoderSetpoint() - s_Arm.getCANCoderPosition()) < 15));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("mech state", inState());
    }
}
