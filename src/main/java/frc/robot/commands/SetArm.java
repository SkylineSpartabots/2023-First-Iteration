package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class SetArm extends CommandBase {
	Arm s_Arm;
	Arm.ArmStates armState;

	public SetArm(Arm.ArmStates armState) {
		s_Arm = Arm.getInstance();
		addRequirements(s_Arm);
		this.armState = armState;
	}

	@Override
	public void initialize() {
		s_Arm.setPosition(armState);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
	
}
