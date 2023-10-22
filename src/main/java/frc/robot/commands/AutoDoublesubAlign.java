/*
 auto balance command for balancing the robot on the charged up station
*/

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AutoDoublesubAlign extends CommandBase {
    Swerve s_Swerve;
    Limelight s_Limelight;
    // double robotPitch;
    double rot;
    // double driveSpeed;
    PIDController turnController = new PIDController(-0.1, 0, 0);
    // double direction = 1.0;
    boolean finished = false;
    // Timer timer = new Timer();

    public AutoDoublesubAlign() {
        s_Swerve = Swerve.getInstance();
        s_Limelight = Limelight.getInstance();
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        // direction = Math.copySign(direction, robotPitch);
        // timer.reset();
        // timer.start();
    }

    @Override
    public void execute() {
        if(s_Limelight.hasTarget() || s_Limelight.getYaw() > 5){
            rot = turnController.calculate(s_Limelight.getYaw(), 0);
            s_Swerve.drive(
                    new Translation2d(0, 0).times(Constants.SwerveConstants.maxSpeed),
                    rot,
                    false,
                    true);
        }
         else {
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(
                new Translation2d(0, 0.1).times(Constants.SwerveConstants.maxSpeed),
                0,
                true,
                true);
    }
}