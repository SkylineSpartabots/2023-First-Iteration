package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    Swerve s_Swerve;
    double robotPitch;
    double driveSpeed;
    PIDController driveController = new PIDController(-0.015, 0, 0); // tune PID
    int counter = 0;
    double direction = 1.0;
    boolean finished = false;

    public AutoBalance() {
        s_Swerve = Swerve.getInstance();
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        direction = Math.copySign(direction, robotPitch);
    }

    @Override
    public void execute() {
        counter++;
        if (counter % 20 == 0) {
            if(Math.abs(s_Swerve.getPitch())+3 >= Math.abs(robotPitch)) {
                driveSpeed = driveController.calculate(s_Swerve.getPitch(), 0);
                SmartDashboard.putNumber("drive speed", driveSpeed);
                SmartDashboard.putNumber("last pit", Math.abs(robotPitch));
                SmartDashboard.putNumber("curr pit 3", Math.abs(s_Swerve.getPitch())+3);
                s_Swerve.drive(
                    new Translation2d(driveSpeed, 0).times(Constants.SwerveConstants.maxSpeed),
                    0,
                    true,
                    true);
            } else {
                finished = true;
            }
            robotPitch = s_Swerve.getPitch();
        }
        
        // driveSpeed = driveController.calculate(robotPitch, 0);
        // SmartDashboard.putNumber("drive speed", driveSpeed);
        // s_Swerve.drive(
        // new Translation2d(driveSpeed, 0).times(Constants.SwerveConstants.maxSpeed),
        // 0,
        // true,
        // true);
        // }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(
                new Translation2d(0, 0).times(Constants.SwerveConstants.maxSpeed),
                0,
                true,
                true);
    }

}