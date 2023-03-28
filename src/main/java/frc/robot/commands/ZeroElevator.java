/*
 zero elevator command to move elevator back down and reset CANCoder position
*/

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
        // sets elevator voltage to a constant low voltage to make it move down
        s_Elevator.setState(ElevatorStates.REALZERO);
        s_Elevator.setVoltage(downVoltage);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        // one the command ends it resets the elavtor CANCoder position to 0
        if (!interrupted) {
            s_Elevator.setCANCoderPosition(0);
        }
        s_Elevator.setVoltage(0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        // when current spike is detected, basically when it is all the
        // way back in and hits the robot, the command ends
        if (s_Elevator.getCurrent() > currentThreshold) {
            return true;
        }
        return false;
    }
}
