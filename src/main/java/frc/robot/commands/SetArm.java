package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;

public class SetArm extends CommandBase {
	Arm s_Arm;
	Arm.ArmStates state;
	double armSpeed;
	PIDController elevatorContoller = new PIDController(0.01, 0, 0);  // tune PID

	SetArm(ArmStates state) {
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
		armSpeed = elevatorContoller.calculate(s_Arm.getCANCoderPosition(), s_Arm.getPositionSetpoint());
		s_Arm.setVelocity(armSpeed);
	}

	@Override
	public boolean isFinished() {
		return Math.abs(s_Arm.getCANCoderPosition() - s_Arm.getPositionSetpoint()) < 150;
	}
}
