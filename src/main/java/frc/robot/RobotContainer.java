package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    // private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton auto = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton smartPathing  = new JoystickButton(driver, XboxController.Button.kB.value);

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    // private final Limelight s_Limelight = Limelight.getInstance();
    
    /* Commands */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.resetOdometry(new Pose2d());
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis)));
                        // () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));
        auto.onTrue(AutoCommandFactory.getSelectedAuto()); // change based on which auto needs to be tested
        // smartPathing.onTrue(new SequentialCommandGroup(
        //     // new SmartResetOdometry(), 
        //     new OnTheFlyGeneration(
        //         s_Swerve.getPose(), 
        //         Constants.Limelight.gameAprilTags2d[s_Limelight.getBestTarget().getFiducialId()-1])
        // ));
        // smartPathing.onTrue(new OnTheFlyGeneration(
        //     s_Swerve.getPose(), 
        //     s_Swerve.getPose().plus(new Transform2d(new Translation2d(-0.5, 0), new Rotation2d()))));
        smartPathing.onTrue(new OnTheFlyGeneration(
            4
        ));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand(String auto) {
    //     // An ExampleCommand will run in autonomous
    //     return AutoCommandFactory.getAutoCommand(auto);
    // }
}
