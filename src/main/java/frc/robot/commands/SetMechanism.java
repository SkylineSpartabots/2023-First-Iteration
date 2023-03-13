package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompleteMechanism;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SetMechanism extends CommandBase {

    private MechanismState state;
    private CompleteMechanism s_Mechanism;

    public SetMechanism(MechanismState state) {
        this.state = state;
        s_Mechanism = CompleteMechanism.getInstance();
        // s_Mechanism.setState(state);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new SetArm(state.armState),
                        new SetElevator(state.elevState)));

        s_Mechanism.setState(state);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return s_Mechanism.inState();
    }
}
