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

    // @Override
    // public void end(boolean interrupted) {
    //     if(state == IntakeStates.ON_RETRACTED){
    //         s_Intake.setState(IntakeStates.OFF_RETRACTED);
    //     } else if (state == IntakeStates.ON_DEPLOYED){
    //         s_Intake.setState(IntakeStates.OFF_DEPLOYED);
    //     }
    // }

    @Override
    public boolean isFinished() {
        // if(state == IntakeStates.ON_RETRACTED || state == IntakeStates.ON_DEPLOYED){
        //     if (s_Intake.hasGamePiece()){
        //         return true;
        //     }
        //     return false;
        // }
        return true;
        // return true;
    }
}
