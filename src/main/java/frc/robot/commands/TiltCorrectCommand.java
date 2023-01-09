package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TiltCorrectCommand extends CommandBase {
    protected DrivetrainSubsystem m_drivetrainSubsystem;
    private PIDController m_thetaController = new PIDController(1, 0, 0);
    public boolean finished;
    public TiltCorrectCommand(boolean isFinished) {
        this.finished = isFinished;
    }

    @Override
    public void execute() {
        // uses pid to adjust wheels accordingly when tilted
        // still need to work out the 
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0, m_thetaController.calculate(Constants.gyro.getPitch(), 0), 0));
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    public void end() {
        // stop
    }
}