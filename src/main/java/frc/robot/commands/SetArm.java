package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;

public class SetArm extends CommandBase {
	Arm s_Arm;
	Arm.ArmStates state;
	double armVoltage;
	// PIDController armController = new PIDController(0.025, 2.5e-3, 0.0);  // tune PID
	ProfiledPIDController armController = new ProfiledPIDController(
		0.025, 2.5e-3, 0, 
		new TrapezoidProfile.Constraints(5, 5)
	);

	public SetArm(ArmStates state) {
		s_Arm = Arm.getInstance();
		addRequirements(s_Arm);
		this.state = state;
	}

	@Override
	public void initialize() {
		s_Arm.setState(state);
	}

	@Override
	public void execute() {
		armVoltage = armController.calculate(s_Arm.getCANCoderPosition(), s_Arm.getPositionSetpoint());
		s_Arm.setVoltage(armVoltage);
	}

	@Override
	public boolean isFinished() {
		// return Math.abs(s_Arm.getCANCoderPosition() - s_Arm.getPositionSetpoint()) < 2;
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		s_Arm.setVoltage(0);
	}
}
