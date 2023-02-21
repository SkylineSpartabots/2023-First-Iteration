package frc.robot.commands;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
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
	// PIDController elevatorController = new PIDController(0.028, 10e-3, 0.0); // tune PID
	ProfiledPIDController elevatorController = new ProfiledPIDController(
		0.050, 1e-2, 0.0,
		// 0.050, 5e-3, 0.0,

		new TrapezoidProfile.Constraints(3200, 3200)
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
		// elevatorController.reset();
	}

	@Override
	public void execute() {
		// s_Elevator.setVoltage(0.7);
		elevatorVoltage = elevatorController.calculate(s_Elevator.getCANCoderPosition(), s_Elevator.getCANCoderSetpoint());
		// elevatorFeedforwardVolage = elevatorFeedforward.calculate(0);
		// s_Elevator.setVoltage(elevatorVoltage + elevatorFeedforwardVolage);
		// if (Math.abs(elevatorVoltage) < 0.46718) {
		// 	elevatorVoltage = 0.46718;
		// }
		// if (Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getCANCoderSetpoint()) > 10) {
			s_Elevator.setVoltage( elevatorVoltage);
		// }
		// if (Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getCANCoderSetpoint()) < 5) {
		// 	elevatorController.reset();
		// }
	}

	@Override
	public boolean isFinished() {
		return true;
	}
		
	@Override
	public void end(boolean interrupted) {
		s_Elevator.setVelocity(0);
	}
}
