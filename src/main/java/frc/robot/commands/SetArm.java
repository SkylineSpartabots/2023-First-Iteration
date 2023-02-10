package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;

public class SetArm extends CommandBase {
	Arm s_Arm;
	Arm.ArmStates state;
	double armVoltage;
	double armFeedforwardVoltage = 0;
	PIDController armController = new PIDController(0.030, 2.5e-3, 0.0); // tune PID
	ArmFeedforward armFeedforward = new ArmFeedforward(0.2782, 0.13793, 0.0025705, 0.00053547);

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
		s_Arm.setVoltage(armVoltage + armFeedforwardVoltage);
		if (Math.abs(s_Arm.getCANCoderPosition() - s_Arm.getPositionSetpoint()) < 5) {
			armController.reset();
		}
	}

	@Override
	public boolean isFinished() {
		// return Math.abs(s_Arm.getCANCoderPosition() - s_Arm.getPositionSetpoint()) <
		// 2;
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		s_Arm.setVoltage(0);
	}
}