package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Pivot;

public class SetArm extends CommandBase {
	Extension s_Extension = Extension.getInstance();
	Pivot s_Pivot = Pivot.getInstance();
	Extension.ExtensionStates extensionState;
	Pivot.PivotStates pivotState;

	public SetArm(Extension.ExtensionStates extensionState, Pivot.PivotStates pivotState) {
		this.extensionState = extensionState;
		this.pivotState = pivotState;
	}

	@Override
	public void initialize() {
		s_Extension.setExtensionPosition(extensionState);
		s_Pivot.setPivotPosition(pivotState);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
	
}
