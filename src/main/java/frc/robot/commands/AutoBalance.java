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
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Light;

  

public class AutoBalance extends CommandBase {
    
    //lights yippie
    private final Light s_Lights = Light.getInstance();

    Swerve s_Swerve;
    double robotPitch;
    double driveSpeed;
    PIDController driveController = new PIDController(-0.015, 0, 0);
    double direction = 1.0;
    boolean finished = false;
    Timer timer = new Timer();

    public AutoBalance() {
        s_Swerve = Swerve.getInstance();
        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {
        direction = Math.copySign(direction, robotPitch);
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // basically checks to see if the angle of the charged up sation has decreased
        // 0.135 degrees
        // since the last time the loop ran (20 ms ago) so bascially when the rate of
        // change is 0.135 dgrees / 20 ms
        // or 6.5 degrees/sec, then stop moving the robot forward
        if (Math.abs(s_Swerve.getPitch()) + 0.135 >= Math.abs(robotPitch) || timer.get() < 1.2) {
            driveSpeed = driveController.calculate(s_Swerve.getPitch(), 0);
            SmartDashboard.putNumber("drive speed", driveSpeed);
            SmartDashboard.putNumber("last pit", Math.abs(robotPitch));
            SmartDashboard.putNumber("curr pit 3", Math.abs(s_Swerve.getPitch()) + 3);
            s_Swerve.drive(
                    new Translation2d(driveSpeed, 0).times(Constants.SwerveConstants.maxSpeed),
                    0,
                    false,
                    true);
        } else {
            finished = true;
        }
        robotPitch = s_Swerve.getPitch();
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        // applies sideway veloctiy to the wheels for 0.1 seconds to lock them sideways
        // and prevent sliding
        timer.stop();
        robotPitch = 0;
        s_Swerve.drive(
                new Translation2d(0, 0.1).times(Constants.SwerveConstants.maxSpeed),
                0,
                true,
                true);
        CommandScheduler.getInstance().schedule(new WaitCommand(0.08).andThen(new InstantCommand(() -> s_Swerve.drive(
                new Translation2d(0, 0).times(Constants.SwerveConstants.maxSpeed),
                0,
                false,
                true))));
    }
}