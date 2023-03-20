package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class ForwardUntilCommand extends CommandBase{
    
    private double driveSpeed;
    private final Swerve s_Swerve;

    public ForwardUntilCommand(){
        s_Swerve = Swerve.getInstance();

    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("forward until running", true);
        driveSpeed = 0.4;
    }

    @Override
    public void execute() {
        s_Swerve.drive(
            new Translation2d(driveSpeed, 0).times(Constants.SwerveConstants.maxSpeed),
        0,
        false,
        true);
        
    }

    @Override
    public boolean isFinished() {
        return Intake.getInstance().hasCube() || s_Swerve.getPose().getX() < 7;
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("forward until running", false);
        s_Swerve.drive(new Translation2d(0, 0), 0, false, true);
    }
    
}
