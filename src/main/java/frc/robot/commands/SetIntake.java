package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntake extends CommandBase {
    Intake.IntakeState state;

    SetIntake(Intake.IntakeState state) {
        this.state = state;
    }

    public void initialize() {
        Intake.getInstance().setState(state);
    }    

    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
