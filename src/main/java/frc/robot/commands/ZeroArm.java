package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;

public class ZeroArm extends CommandBase {
    Arm s_Arm;
    final double currentThreshold = 20;
    final double downVoltage = -3;

    public ZeroArm() {
        s_Arm = Arm.getInstance();
        addRequirements(s_Arm);
    }

    @Override
    public void initialize() {
        s_Arm.setState(ArmStates.REALZERO);
        s_Arm.setVoltage(downVoltage);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        s_Arm.setVoltage(0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        if (s_Arm.getCurrent() > currentThreshold) {
            return true;
        }
        return false;
    }
}
