package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntake extends CommandBase {
    Intake s_Intake;
    Intake.IntakeStates state;

    public SetIntake(Intake.IntakeStates state) {
        s_Intake = Intake.getInstance();        
        this.state = state;
    }

    public void initialize() {
        s_Intake.setState(state);
    }    

    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
