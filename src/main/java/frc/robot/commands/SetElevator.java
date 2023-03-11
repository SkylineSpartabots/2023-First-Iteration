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
	// PIDController elevatorController = new PIDController(0.028, 10e-3, 0.0); // tune PID
	// ProfiledPIDController downController = new ProfiledPIDController(
	// 	0.050, 1e-2, 1e-3,
	// 	new TrapezoidProfile.Constraints(1e9, 1e9)
	// );
	ProfiledPIDController upController = new ProfiledPIDController(
		0.050, 1e-2, 1e-3,
		// 0.020, 0, 0,
		new TrapezoidProfile.Constraints(3000, 3000)
	);
	ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0.083319, 0.46718, 62.909, 3.709);

	public SetElevator(ElevatorStates state) {
		s_Elevator = Elevator.getInstance();
		addRequirements(s_Elevator);
		this.state = state;
	}

	@Override
	public InterruptionBehavior getInterruptionBehavior() {
		return InterruptionBehavior.kCancelSelf;
	}

	@Override
	public void initialize() {
		s_Elevator.setState(state);
		upController.reset(s_Elevator.getCANCoderPosition());
		// upController.reset(s_Elevator.getCANCoderPosition());
		// elevatorController.reset();
	}

	@Override
	public void execute() {
		elevatorVoltage = upController.calculate(s_Elevator.getCANCoderPosition(), s_Elevator.getCANCoderSetpoint());
		if (Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getCANCoderSetpoint()) < 15) {
			elevatorVoltage = 0.8;
		}
		s_Elevator.setVoltage( elevatorVoltage);
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
