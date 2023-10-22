/* 
 has all the button initilizations and subsystem initlizations
*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.CompleteMechanism.MechanismState;
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

    /* Driver Joysticks (drive control) */
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

    /* Operator Buttons */
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

    /* Operator Joysticks */
    private final int operatorLeftStick = XboxController.Axis.kLeftY.value;
    private final int operatorRightStick = XboxController.Axis.kRightY.value;

    /* Subsystems */
    private final Swerve s_Swerve;
    private final Limelight s_Limelight;
    private final Elevator s_Elevator;
    private final Arm s_Arm;
    private final Intake s_Intake;
    private final CompleteMechanism s_CompleteMechanism;
    private final AutomaticScoringSelector selector;

    private boolean cone = true;

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
        selector = AutomaticScoringSelector.getInstance();
        selector.createDisplay();


        s_Swerve.resetOdometry(new Pose2d());
        s_Swerve.zeroGyro();
        // sets the teleop swerve command as default command with the input from driver joysticks
        // to control the swerve
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis)));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // driver controls
        driverBack.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));
        driverStart.onTrue(new SmartResetOdometry());

        driverRightBumper.onTrue(new InstantCommand(() -> groundIntake()));
        driverRightTrigger.onTrue(new InstantCommand(() -> doubleSubIntake()));
        driverLeftTrigger.onTrue(new InstantCommand(() -> singleSubIntake()));
        driverLeftBumper.onTrue(new InstantCommand(() -> {display(); cone = !cone;}));

        driverDpadDown.onTrue(new InstantCommand(() -> s_CompleteMechanism.l1State()));
        driverDpadRight.onTrue(new InstantCommand(() -> s_CompleteMechanism.l2State()));
        driverDpadUp.onTrue(l3());
        // driverDpadLeft.onTrue(new AutoBalance());

        driverA.onTrue(new InstantCommand(() -> zeroMech()));
        driverX.onTrue(new InstantCommand(() -> reverseIntake()));
        driverB.onTrue(new ZeroElevator());
        driverY.onTrue(onIntake());

        // operator controls
        operatorStart.onTrue(new SmartResetOdometry());
        operatorBack.onTrue(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d())));

        operatorDpadRight.onTrue(new InstantCommand(() -> selector.increasePos()));
        operatorDpadLeft.onTrue(new InstantCommand(() -> selector.decreasePos()));

        operatorRightTrigger.onTrue(new InstantCommand(() -> selector.setLevel(2)));
        operatorLeftTrigger.onTrue(new InstantCommand(() -> selector.setLevel(1)));
        operatorLeftBumper.onTrue(new InstantCommand(() -> selector.setLevel(0)));

        operatorRightBumper.onTrue(new AutoTeleopScore());
    }

    // the methods below abstract functionality of the subsystems to make it easier for driver control
    // uses the cone boolean to set the states in either cube or cone mode based on which is selcted
    

    public Command l3() {
        if (cone == true) {
            return new SetMechanism(MechanismState.L3CONE);
        }
        return new SetMechanism(MechanismState.L3CUBE);
    }

    public void display() {
        SmartDashboard.putBoolean("cone", cone);
    }

    public Command onIntake() {
        if (cone == true) {
            return new SetIntake(IntakeStates.ON_CONE);
        } else {
            return new SetIntake(IntakeStates.ON_CUBE);
        }
        
    }

    public Command offIntake() {
        return new SetIntake(IntakeStates.OFF);
    }

    public void reverseIntake() {
        if (cone) {
            CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.REV_CONE));
        } else {
            //if(s_CompleteMechanism.getState()) //TODO: get this to increase Intake power based on what Mechanism state should be.
            CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.REV_CUBE));
        }
    }

    public void groundIntake() {
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new SetMechanism(MechanismState.GROUNDINTAKE),
                        onIntake()));
    }

    public void singleSubIntake() {
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new SetMechanism(MechanismState.SUBSTATION),
                        onIntake()));
    }

    public void doubleSubIntake() {
        if (cone) {
            CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                            new SetMechanism(MechanismState.CONEDOUBLESUBSTATION),  
                            onIntake()));
        } else {
            CommandScheduler.getInstance().schedule(
                    new ParallelCommandGroup(
                            new SetMechanism(MechanismState.CUBEDOUBLESUBSTATION),
                            onIntake()));
        }
    }

    public void zeroMech() {
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new SetMechanism(MechanismState.ZERO),
                        offIntake()));
    }

    // does not work
    public void onRobotDisabled() {
        // reset mechanisms so it doesnot have to be done manually
        CommandScheduler.getInstance().schedule(new SetArm(ArmStates.ZERO));
        CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF));
    }
}