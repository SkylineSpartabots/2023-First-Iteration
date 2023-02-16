package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class SetElevator extends CommandBase {
	Elevator s_Elevator;
	Elevator.ElevatorStates state;
	double elevatorVoltage;
	double elevatorFeedforwardVolage;
	PIDController elevatorController = new PIDController(0.025, 2.5e-3, 0.0); // tune PID
	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0);

	public SetElevator(ElevatorStates state) {
		s_Elevator = Elevator.getInstance();
		addRequirements(s_Elevator);
		this.state = state;
	}

	@Override
	public void initialize() {
		s_Elevator.setState(state);
		elevatorController.reset();
	}

	@Override
	public void execute() {
		elevatorVoltage = elevatorController.calculate(s_Elevator.getCANCoderPosition(), s_Elevator.getCANCoderSetpoint());
		elevatorFeedforwardVolage = elevatorFeedforward.calculate(0);
		s_Elevator.setVoltage(elevatorVoltage + elevatorFeedforwardVolage);
		// if (Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getCANCoderSetpoint()) < 5) {
		// 	elevatorController.reset();
		// }
	}

	@Override
	public boolean isFinished() {
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		s_Elevator.setVelocity(0);
	}
}
