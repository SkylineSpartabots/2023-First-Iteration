package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class SetArm extends CommandBase {
	Elevator s_Elevator;
	Arm s_Arm;
	Elevator.ElevatorStates elevatorState;
	Arm.ArmStates armState;

	public SetArm(Elevator.ElevatorStates elevatorState, Arm.ArmStates armState) {
		s_Elevator = Elevator.getInstance();
		s_Arm = Arm.getInstance();
		addRequirements(s_Elevator, s_Arm);
		this.elevatorState = elevatorState;
		this.armState = armState;
	}

	@Override
	public void initialize() {
		s_Elevator.setPosition(elevatorState);
		s_Arm.setPosition(armState);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
	
}
