package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CompleteMechanism;
import frc.robot.subsystems.CompleteMechanism.MechanismState;

public class SetMechanism extends CommandBase{

    private MechanismState state;
    private CompleteMechanism mech;
    
    public SetMechanism(MechanismState state){
        this.state = state;
        mech = CompleteMechanism.getInstance();
    }
    
    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        SetArm armCommand = new SetArm(state.armState); armCommand.schedule();
        SetElevator elevCommand = new SetElevator(state.elevState); elevCommand.schedule();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return state == mech.getState();
    }
}
