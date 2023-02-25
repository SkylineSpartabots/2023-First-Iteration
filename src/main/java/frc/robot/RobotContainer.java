package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.factories.ScoringCommandFactory;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Intake.IntakeStates;

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
    private final XboxController driver = new XboxController(0);
    private final XboxController operator = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton driverBack = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton driverStart = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driverY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driverRightBumper = new JoystickButton(driver,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton driverLeftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final Trigger driverDpadUp = new Trigger(() -> driver.getPOV() == 0);
    private final Trigger driverDpadRight = new Trigger(() -> driver.getPOV() == 90);
    private final Trigger driverDpadDown = new Trigger(() -> driver.getPOV() == 180);
    private final Trigger driverDpadLeft = new Trigger(() -> driver.getPOV() == 270);
    private final Trigger driverLeftTrigger = new Trigger(() -> {
        return driver.getLeftTriggerAxis() > Constants.triggerDeadzone;
    });
    private final Trigger driverRightTrigger = new Trigger(() -> {
        return driver.getRightTriggerAxis() > Constants.triggerDeadzone;
    });

    /* Operator Buttons, currently just used for testing */
    private final JoystickButton operatorBack = new JoystickButton(operator, XboxController.Button.kBack.value);
    private final JoystickButton operatorStart = new JoystickButton(operator, XboxController.Button.kStart.value);
    private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorRightBumper = new JoystickButton(operator,
            XboxController.Button.kRightBumper.value);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operator,
            XboxController.Button.kLeftBumper.value);

    private final Trigger operatorDpadUp = new Trigger(() -> operator.getPOV() == 0);
    private final Trigger operatorDpadRight = new Trigger(() -> operator.getPOV() == 90);
    private final Trigger operatorDpadDown = new Trigger(() -> operator.getPOV() == 180);
    private final Trigger operatorDpadLeft = new Trigger(() -> operator.getPOV() == 270);
    private final Trigger operatorLeftTrigger = new Trigger(() -> {
        return operator.getLeftTriggerAxis() > Constants.triggerDeadzone;
    });
    private final Trigger operatorRightTrigger = new Trigger(() -> {
        return operator.getRightTriggerAxis() > Constants.triggerDeadzone;
    });

    /* Subsystems */
    private final Swerve s_Swerve;
    private final Limelight s_Limelight;
    private final Elevator s_Elevator;
    private final Arm s_Arm;
    private final Intake s_Intake;
    private final ScoringCommandFactory scoreCommandFactory;
    private final AutomaticScoringSelector selector;
    private boolean currentlyCoast;

    /* Commands */

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // initialize subsystems
        s_Swerve = Swerve.getInstance();
        s_Limelight = Limelight.getInstance();
        s_Elevator = Elevator.getInstance();
        s_Intake = Intake.getInstance();
        s_Arm = Arm.getInstance();
       scoreCommandFactory = ScoringCommandFactory.getInstance();
       selector = AutomaticScoringSelector.getInstance();
       selector.createDisplay();

        // s_Swerve.resetOdometry(new Pose2d());
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
        driverBack.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));
        driverStart.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));
        // driverA.onTrue(AutoCommandFactory.getSelectedAuto()); // change based on
        // which auto needs to be tested
        // driverB.onTrue(new ConditionalCommand(
        // new InstantCommand(() -> AutoCommandFactory.cancelLastCommand()),
        // new InstantCommand(() -> CommandScheduler.getInstance().schedule(new
        // OnTheFlyGeneration(0, true))),
        // s_Swerve.isPathRunningSupplier));
        // driverX.onTrue(new SmartResetOdometry());
        // Command autoBalanceCommand = new AutoBalance();
        // driverY.onTrue(new ConditionalCommand(
        // autoBalanceCommand,
        // new InstantCommand(() -> autoBalanceCommand.cancel()),
        // s_Swerve.isPathRunningSupplier));
        // operatorA.onTrue(new InstantCommand(() -> s_Arm.changePosition(true)));
        // operatorY.onTrue(new InstantCommand(() -> s_Arm.changePosition(false)));
        // elevatorUp.onTrue(new InstantCommand(() -> s_Elevator.changePosition(true)));
        // elevatorDown.onTrue(new InstantCommand(() ->
        // s_Elevator.changePosition(false)));
        // setArm.onTrue(new SetArm(Extension.ExtensionStates.ZERO,
        // Pivot.PivotStates.ZERO));
        // setPivot.onTrue(new Instant5tCommand(() ->
        // s_Intake.setState(IntakeStates.ON_DEPLOYED)));
        // operatorX.onTrue(new InstantCommand(() -> s_Elevator.setVelocity(-1000)));
        // operatorB.onTrue(new InstantCommand(() -> s_Elevator.setVelocity(1000)));
        // operatorA.onTrue(new SetElevator(ElevatorStates.GROUND));
        driverA.onTrue(new InstantCommand(() -> s_Arm.toggleNeutral()));
        driverB.onTrue(new SetIntake(IntakeStates.REV_DEPLOYED));
        driverX.onTrue(new SetIntake(IntakeStates.REV_RETRACTED));
        driverY.onTrue(new SmartResetOdometry());
        // driverB.onTrue(scoreCommandFactory.getScoringCommand(null, null))
        // driverX.onTrue(new SetArm(ArmStates.L1));
        // driverY.onTrue(new SetArm(ArmStates.SUBSTATION));
        driverLeftBumper.onTrue(new SetIntake(IntakeStates.OFF_RETRACTED));
        driverLeftTrigger.onTrue(new SetIntake(IntakeStates.OFF_DEPLOYED));
        driverRightBumper.onTrue(new SetIntake(IntakeStates.ON_RETRACTED));
        driverRightTrigger.onTrue(new SetIntake(IntakeStates.ON_DEPLOYED));
        // driverLeftTrigger.onTrue(new SetIntake(IntakeStates.OFF_RETRACTED));
        // driverRightTrigger.onTrue(new SetIntake(IntakeStates.ON_RETRACTED));
        // driverA.onTrue(new SetArm(ArmStates.CONE));
        // driverB.onTrue(new SetArm(ArmStates.ZERO));
        // driverDpadDown.onTrue(new SetElevator(ElevatorStates.ZERO));
        // driverDpadUp.onTrue(new SetElevator(ElevatorStates.L1CONE));
        // driverDpadRight.onTrue(new SetElevator(ElevatorStates.SUBSTATION));
        // driverDpadLeft.onTrue(new SetElevator(ElevatorStates.L3CONE));
        driverDpadDown.onTrue(new SetMechanism(MechanismState.CUBEINTAKE));
        driverDpadUp.onTrue(new SetMechanism(MechanismState.ZERO));
        // driverDpadRight.onTrue(new SetMechanism(MechanismState.CONEINTAKE));
        // driverDpadLeft.onTrue(new SetMechanism(MechanismState.MIDCUBE));
        driverDpadRight.onTrue(new SetElevator(ElevatorStates.L2CUBE));
        driverDpadLeft.onTrue(new SetArm(ArmStates.ZERO));

        operatorDpadUp.onTrue(new InstantCommand(() -> selector.moveUp()));
        operatorDpadDown.onTrue(new InstantCommand(() -> selector.moveDown()));
        operatorDpadRight.onTrue(new InstantCommand(() -> selector.moveRight()));
        operatorDpadLeft.onTrue(new InstantCommand(() -> selector.moveLeft()));
        operatorA.onTrue(new InstantCommand(() -> selector.select()));
        operatorB.onTrue(new SmartPathGenerating(s_Swerve.getPose(), selector.getSelectedPose()));
        operatorX.onTrue(new OnTheFlyGeneration(s_Swerve.getPose(), selector.getSelectedPose(), true));

        // setIntake.onTrue(new InstantCommand(() -> s_Elevator.setPos(0)));
        // -85 bottom
        // 96 top

        DriverStation.Alliance alliance = DriverStation.getAlliance();

        final int[] driveTags;
        {
            // up gets you to substation, left/down/right will get you to the
            // left/middle/right grids individually
            // maybe can be further optimized to shift slightly left or right if bot has
            // cone
            if (alliance == DriverStation.Alliance.Red) {
                driveTags = new int[] { 3, 5, 6, 7 }; // see 5.9.2 in game manual
            } else { // defaults to blue alliance if alliance is not set for whatever reason
                driveTags = new int[] { 4, 1, 1, 2 };
            }
            // operatorDpadUp.onTrue(new OnTheFlyGeneration(driveTags[0], true)); // fixme
            // whats the POINT of swervePose why cant we call it in the function itself
            // operatorDpadLeft.onTrue(new OnTheFlyGeneration(driveTags[1], true));
            // operatorDpadDown.onTrue(new OnTheFlyGeneration(driveTags[2], true));
            // operatorDpadRight.onTrue(new OnTheFlyGeneration(driveTags[3], true));
        }
    }

    public void onRobotDisabled() {
        // reset mechanisms so it does not have to be done manually
        CommandScheduler.getInstance().schedule(new SetArm(ArmStates.ZERO));
        CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED));
    }

}