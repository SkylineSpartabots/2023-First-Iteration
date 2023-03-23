package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class SetElevator extends CommandBase {
	Elevator s_Elevator;
	Elevator.ElevatorStates state;
	double elevatorVoltage;
	double elevatorFeedforwardVolage;
	ProfiledPIDController elevatorController = new ProfiledPIDController(
		0.060, 1e-2, 1e-3,
		new TrapezoidProfile.Constraints(500000, 3000 * 1e5)
	);
	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.083319, 0.46718, 62.909, 3.709);

	public SetElevator(ElevatorStates state) {
		s_Elevator = Elevator.getInstance();
		addRequirements(s_Elevator);
		this.state = state;
	}

	@Override
	public void initialize() {
		s_Elevator.setState(state);
		elevatorController.reset(s_Elevator.getCANCoderPosition());
	}

	@Override
	public void execute() {
		elevatorVoltage = elevatorController.calculate(s_Elevator.getCANCoderPosition(), s_Elevator.getCANCoderSetpoint());
		if (Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getCANCoderSetpoint()) < 15) {
			elevatorVoltage = 0.7;
		}
		s_Elevator.setVoltage(elevatorVoltage);
	}

	@Override
	public boolean isFinished() {
		return s_Elevator.elevatorError();
	}
		
	@Override
	public void end(boolean interrupted) {
		s_Elevator.setVoltage(0);
	}
}
