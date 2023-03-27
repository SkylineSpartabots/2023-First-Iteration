/*
 set intake command to control intake arm mechanism
*/

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
        // calls the method with the new intake state that will change it
        s_Intake.setState(state);
    }    

    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
