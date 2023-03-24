// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SetMechanism;
import frc.robot.factories.AutoCommandFactory;
import frc.robot.subsystems.CompleteMechanism.MechanismState;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static CTREConfigs ctreConfigs;

    private Command m_autonomousCommand;
    private final SendableChooser<AutoCommandFactory.AutoType> m_chooser = new SendableChooser<>();
    private RobotContainer m_robotContainer;
    // private AutomaticScoringSelector m_selector =
    // AutomaticScoringSelector.getInstance();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        getRedPose();
        ctreConfigs = new CTREConfigs();
        m_chooser.addOption("Test", AutoCommandFactory.AutoType.Test);
        m_chooser.addOption("1C", AutoCommandFactory.AutoType.OneCone);
        m_chooser.addOption("1C B Back", AutoCommandFactory.AutoType.OneConeBackBottom);
        m_chooser.addOption("1C M Dock", AutoCommandFactory.AutoType.OneConeDockMiddle);
        m_chooser.addOption("1C M Dock Com", AutoCommandFactory.AutoType.OneConeDockMiddleCom);
        m_chooser.addOption("1.5C T Dock", AutoCommandFactory.AutoType.OneHalfConeDockTop);
        m_chooser.addOption("2C B", AutoCommandFactory.AutoType.TwoConeBottom);
        m_chooser.addOption("2C B Dock", AutoCommandFactory.AutoType.TwoConeDockBottom);
        m_chooser.addOption("2C T Dock", AutoCommandFactory.AutoType.TwoConeDockTop);
        m_chooser.setDefaultOption("2C T", AutoCommandFactory.AutoType.TwoConeTop);
        m_chooser.addOption("3C T", AutoCommandFactory.AutoType.ThreeConeTop);
        SmartDashboard.putData("Auto choices", m_chooser);
        DriverStation.Alliance a = DriverStation.getAlliance();
        SmartDashboard.putString("Alliance",
                a == DriverStation.Alliance.Blue ? "Blue" : a == DriverStation.Alliance.Red ? "Red" : "Other");
        m_robotContainer = new RobotContainer();
        CommandScheduler.getInstance().schedule(new SetMechanism(MechanismState.ZERO));
    }

    public static void getRedPose() {
        for (int i = 0; i < 8; i++) {
            Pose3d a = Constants.Limelight.blueGameAprilTags[i];
            Constants.Limelight.redGameAprilTags[i] = new Pose3d(Constants.FIELD_HEIGHT_METERS - a.getX(),
                    Constants.FIELD_WIDTH_METERS - a.getY(), a.getZ(), a.getRotation().minus(new Rotation3d(0, 0, Math.PI)));
            Constants.Limelight.redGameAprilTags2d[i] = Constants.Limelight.redGameAprilTags[i].toPose2d();
        }

        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            Constants.Limelight.gameAprilTags = Constants.Limelight.redGameAprilTags;
            Constants.Limelight.gameAprilTags2d = Constants.Limelight.redGameAprilTags2d;
        } else {
            Constants.Limelight.gameAprilTags = Constants.Limelight.blueGameAprilTags;
            Constants.Limelight.gameAprilTags2d = Constants.Limelight.blueGameAprilTags2d;
        }

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and
     * test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        m_robotContainer.onRobotDisabled();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = AutoCommandFactory.getAutoCommand(m_chooser.getSelected());
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        // m_selector.updateShuffleboard();
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
