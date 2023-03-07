package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Arm.ArmStates;

public class ScoringPosition {

    Pose2d targetPos;
    ArmStates armState;
    ElevatorStates elevatorState;

    public ScoringPosition(Pose2d pose, frc.robot.subsystems.Arm.ArmStates armStatesCube, ElevatorStates elevState){
        this.targetPos = pose;
        this.armState = armStatesCube;
        this.elevatorState = elevState;
    }

    public ArmStates getArmState() {
        return armState;
    }

    public ElevatorStates getElevatorState() {
        return elevatorState;
    }

    public Pose2d getTargetPos() {
        return targetPos;
    }
}
