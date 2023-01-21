package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    Swerve s_Swerve = Swerve.getInstance();
    double robotPitch;
    double driveSpeed;
    PIDController driveController = new PIDController(-0.015, 0, 0);  // tune PID

    public AutoBalance() {
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        robotPitch = s_Swerve.getPitch();
        if (Math.abs(robotPitch) < 0.3) {
            robotPitch = 0;
        }
        driveSpeed = driveController.calculate(robotPitch, 0);
        SmartDashboard.putNumber("drive speed", driveSpeed);
        s_Swerve.drive(
            new Translation2d(driveSpeed, 0).times(Constants.Swerve.maxSpeed), 
            0, 
            true, 
            true
        );
    }
    
}
