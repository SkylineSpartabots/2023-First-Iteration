package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    private final JoystickButton auto = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton smartPathing = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton smartOdo = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton autoBalance = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton cancelAutoBalance = new JoystickButton(driver, XboxController.Button.kStart.value);

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Limelight s_Limelight = Limelight.getInstance();
    private final Extension s_Extension = Extension.getInstance();
    private final Pivot s_Pivot = Pivot.getInstance();

    /* Commands */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        s_Swerve.resetOdometry(new Pose2d());
        s_Swerve.zeroGyro();
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
        smartPathing.onTrue(new ConditionalCommand(
                // new InstantCommand(() -> s_Swerve.getCurrentCommand().cancel()),
                new InstantCommand(() -> AutoCommandFactory.cancelLastCommand()),
                new InstantCommand(() -> CommandScheduler.getInstance().schedule(new OnTheFlyGeneration(0, true))),
                s_Swerve.isPathRunningSupplier));
        smartOdo.onTrue(new SmartResetOdometry());
        autoBalance.onTrue(new AutoBalance());
        cancelAutoBalance.onTrue(new InstantCommand(() -> s_Swerve.getCurrentCommand().cancel()));
    }

    public void onRobotDisabled() {
        // reset mechanisms so it does not have to be done manually
    }

}
