package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;

public class ZeroElevator extends CommandBase {
    Elevator s_Elevator;
    final double currentThreshold = 15;
    final double downVoltage = -2.2;

    public ZeroElevator() {
        s_Elevator = Elevator.getInstance();
        addRequirements(s_Elevator);
    }

    @Override
    public void initialize() {
        s_Elevator.setState(ElevatorStates.REALZERO);
        s_Elevator.setVoltage(downVoltage);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            s_Elevator.setCANCoderPosition(0);
        }
        s_Elevator.setVoltage(0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        if (s_Elevator.getCurrent() > currentThreshold) {
            return true;
        }
        return false;
    }
}
