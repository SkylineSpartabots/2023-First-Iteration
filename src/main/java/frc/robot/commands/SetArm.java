/*
 set arm command to control the arm mechanism, is constantly run
*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;

public class SetArm extends CommandBase {
	Arm s_Arm;
	Arm.ArmStates state;
	double armVoltage;
	// pid controller that controls the input to the arm based on current position and goal position
	PIDController armController = new PIDController(0.23, 5.4e-3, 1.6e-3); 

	public SetArm(ArmStates state) {
		s_Arm = Arm.getInstance();
		addRequirements(s_Arm);
		this.state = state;
	}

	@Override
	public void initialize() {
		s_Arm.setState(state);
		armController.reset();
	}

	@Override
	public void execute() {
		// uses PID controller and calcualtes voltage input based on current pos and goal pos
		armVoltage = armController.calculate(s_Arm.getCANCoderPosition(), s_Arm.getCANCoderSetpoint());
		s_Arm.setVoltage(armVoltage);
	}

	@Override
	public boolean isFinished() {
		// constantly run, only ends if an arm error is detected (i.e. CANcode disconnects)
		return s_Arm.armError();
	}

	@Override
	public void end(boolean interrupted) {
		s_Arm.setVoltage(0);
	}
}