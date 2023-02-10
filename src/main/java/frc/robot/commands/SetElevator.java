package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class SetElevator extends CommandBase {
	Elevator s_Elevator;
	Elevator.ElevatorStates state;
	double elevatorSpeed;
	ProfiledPIDController elevatorController = new ProfiledPIDController(
			10, 0, 0,
			new TrapezoidProfile.Constraints(100000000, 10000));

	public SetElevator(ElevatorStates state) {
		s_Elevator = Elevator.getInstance();
		addRequirements(s_Elevator);
		this.state = state;
	}

	@Override
	public void initialize() {
		s_Elevator.setState(state);
	}

	@Override
	public void execute() {
		elevatorSpeed = elevatorController.calculate(s_Elevator.getCANCoderPosition(),
				s_Elevator.getPositionSetpoint());
		s_Elevator.setVelocity(elevatorSpeed);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(s_Elevator.getCANCoderPosition() - s_Elevator.getPositionSetpoint()) < 10;
	}

	@Override
	public void end(boolean interrupted) {
		s_Elevator.setVelocity(0);
	}
}
