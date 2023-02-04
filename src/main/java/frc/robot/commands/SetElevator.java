package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class SetElevator extends CommandBase {
	Elevator s_Elevator;
	Elevator.ElevatorStates state;
	double elevatorSpeed;
	PIDController elevatorContoller = new PIDController(0.01, 0, 0);  // tune PID

	SetElevator(ElevatorStates state) {
		s_Elevator = Elevator.getInstance();
		this.state = state;
	}

	@Override
	public void initialize() {
		s_Elevator.setState(state);
	}

	@Override
	public void execute() {
		elevatorSpeed = elevatorContoller.calculate(s_Elevator.getCANCoderPosition(), s_Elevator.getPositionSetpoint());
		s_Elevator.setVelocity(elevatorSpeed);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getPositionSetpoint()) < 150;
	}
}
