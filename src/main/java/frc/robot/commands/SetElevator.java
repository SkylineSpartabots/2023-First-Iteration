/*
 set elevator command to control the elevator mechanism, is constantly run
*/

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
	// profiled pid controller that controls the input to the elevator based on current position and goal position
	// the profiling limits its max acceleration and velocity to make it smoother
	ProfiledPIDController elevatorController = new ProfiledPIDController(
		0.060, 1e-2, 1e-3,
		new TrapezoidProfile.Constraints(500000, 3000 * 1e5)
	);
	// feedforward was something we did not use because our PID controller was good enough without 
	// it and we applied a constant 0.7 voltage to prevent elevator from falling due to gravity
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
		// uses PID controller and calcualtes voltage input based on current pos and goal pos
		elevatorVoltage = elevatorController.calculate(s_Elevator.getCANCoderPosition(), s_Elevator.getCANCoderSetpoint());
		// if within 15 CANCoder positions of goal pose, set voltage to 0.7 which basically just keeps it at the same place
		if (Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getCANCoderSetpoint()) < 15) {
			elevatorVoltage = 0.7;
		}
		s_Elevator.setVoltage(elevatorVoltage);
	}

	@Override
	public boolean isFinished() {
		// constantly run, only ends if an elevator error is detected (i.e. CANcode disconnects)
		return s_Elevator.elevatorError();
	}
		
	@Override
	public void end(boolean interrupted) {
		s_Elevator.setVoltage(0);
	}
}
