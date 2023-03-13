package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    /* operator joysticks */

    private final int operatorLeftStick = XboxController.Axis.kLeftY.value;
    private final int operatorRightStick = XboxController.Axis.kRightY.value;

    /* Subsystems */
    private final Swerve s_Swerve;
    private final Limelight s_Limelight;
    private final Elevator s_Elevator;
    private final Arm s_Arm;
    private final Intake s_Intake;
    private final CompleteMechanism s_CompleteMechanism;
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
        s_CompleteMechanism = CompleteMechanism.getInstance();
        scoreCommandFactory = ScoringCommandFactory.getInstance();
        selector = AutomaticScoringSelector.getInstance();
        selector.createDisplay();

        s_Swerve.resetOdometry(new Pose2d());
        s_Swerve.zeroGyro();
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis)));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // driver buttons
        driverStart.onTrue(new SmartResetOdometry());
        driverBack.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));
        driverRightTrigger.onTrue(coneSubstation);
        driverLeftTrigger.onTrue(cubeIntake);
        driverRightBumper.onTrue(coneIntake);
        driverB.onTrue(new InstantCommand(() -> zeroCommand()));
        driverA.onTrue(new ZeroElevator());
        // temporary commands (should be on operator)
        driverDpadRight.onTrue(new InstantCommand(() -> selector.increasePos()));
        driverDpadLeft.onTrue(new InstantCommand(() -> selector.decreasePos()));
        driverDpadUp.onTrue(new InstantCommand(() -> selector.setLevel(2)));
        driverDpadDown.onTrue(new InstantCommand(() -> selector.setLevel(0)));
        driverX.onTrue(new InstantCommand(() -> selector.setLevel(1)));
        driverY.onTrue(new AutoTeleopScore());

        // operator buttons
        // operatorA.onTrue(new InstantCommand(() -> s_CompleteMechanism.l1State()));
        // operatorX.onTrue(new InstantCommand(() -> s_CompleteMechanism.l2State()));
        // operatorY.onTrue(new InstantCommand(() -> s_CompleteMechanism.l3State()));
        // operatorB.onTrue(new InstantCommand(() -> reverseCommand()));
        operatorRightTrigger.onTrue(new InstantCommand(() -> s_Intake.setSpeed(0.75)));
        operatorLeftTrigger.onTrue(new InstantCommand(() -> s_Intake.setSpeed(0)));
        operatorDpadRight.onTrue(new InstantCommand(() -> selector.increasePos()));
        operatorDpadLeft.onTrue(new InstantCommand(() -> selector.decreasePos()));
        operatorDpadUp.onTrue(new InstantCommand(() -> selector.setLevel(2)));
        operatorDpadDown.onTrue(new InstantCommand(() -> selector.setLevel(1)));
        operatorLeftBumper.onTrue(new InstantCommand(() -> selector.setLevel(0)));

        operatorA.onTrue(new SetIntake(IntakeStates.OFF_CLOSED_CONE));
        operatorB.onTrue(new SetIntake(IntakeStates.ON_CLOSED_CONE));
        operatorX.onTrue(new SetIntake(IntakeStates.REV_CLOSED_CONE));
        operatorY.onTrue(new SetIntake(IntakeStates.OFF_OPEN_CONE));

        operatorRightBumper.onTrue(new AutoTeleopScore());
    }

    ParallelCommandGroup coneIntake = new ParallelCommandGroup(
            new SetMechanism(MechanismState.CONEINTAKE),
            new SetIntake(IntakeStates.ON_CLOSED_CONE));
    ParallelCommandGroup cubeIntake = new ParallelCommandGroup(
            new SetMechanism(MechanismState.CUBEINTAKE),
            new SetIntake(IntakeStates.ON_OPEN_CUBE));
//     ParallelCommandGroup layedCone = new ParallelCommandGroup(
//             new SetMechanism(MechanismState.LAYEDCONE),
//             new SetIntake(IntakeStates.ON_DEPLOYED_LAYEDCONE));
    ParallelCommandGroup coneSubstation = new ParallelCommandGroup(
            new SetMechanism(MechanismState.DOUBLESUBSTATION),
            new SetIntake(IntakeStates.ON_CLOSED_CONE));

    ParallelCommandGroup zeroCone = new ParallelCommandGroup(
            new SetMechanism(MechanismState.ZERO),
            new SetIntake(IntakeStates.OFF_CLOSED_CONE));
    ParallelCommandGroup zeroCube = new ParallelCommandGroup(
            new SetMechanism(MechanismState.ZERO),
            new SetIntake(IntakeStates.OFF_OPEN_CUBE));
//     ParallelCommandGroup zeroLayed = new ParallelCommandGroup(
//             new SetMechanism(MechanismState.ZERO),
//             new SetIntake(IntakeStates.OFF_RETRACTED_LAYEDCONE));

    public void zeroCommand() {
        CommandScheduler.getInstance().schedule(
                s_Intake.intakeState.piece.equals("cube") ? zeroCube : zeroCone);
    }

    public void reverseCommand() {  
        CommandScheduler.getInstance().schedule(
                        s_Intake.intakeState.piece.equals("cube") ? new SetIntake(IntakeStates.REV_OPEN_CUBE)
                                : new SetIntake(IntakeStates.OFF_OPEN_CONE));
    }

    public void onRobotDisabled() {
        // reset mechanisms so it does not have to be done manually
        CommandScheduler.getInstance().schedule(new SetArm(ArmStates.ZERO));
        CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_OPEN_CONE));
    }
}