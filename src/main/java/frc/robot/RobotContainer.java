// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.lib.util.Controller;
import frc.lib.util.DeviceFinder;
import frc.robot.commands.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem m_drivetrainSubsystem;
  
  /*private static PowerDistribution powerModule = new PowerDistribution(1, ModuleType.kRev);

  public static PowerDistribution getPDP(){
    return powerModule;
  }*/
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   *
   */
  public RobotContainer() {
    m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

    // Set the scheduler to log Shuffleboard events for command initialize,
    // interrupt, finish
    /*CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker(
        "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker(
        "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker(
        "Command finished", command.getName(), EventImportance.kNormal));
*/
    // Configure the button bindings
    configureButtonBindings();

  }

  private static final Controller m_controller = new Controller(new XboxController(0));
  private static final Controller m_controller2 = new Controller(new XboxController(1));

  public static Controller getController(){
    return m_controller;
  }

  public static void printDiagnostics(){
    /*SmartDashboard.putBoolean("NavX Connected?", DrivetrainSubsystem.getInstance().getNavxConnected());
    SmartDashboard.putBoolean("Limelight Connected?", LimelightSubsystem.getInstance().isConnected());
    SmartDashboard.putBoolean("Can Bus Connected?", isCanConnected());
    SmartDashboard.putBoolean("Battery Charged?", isBatteryCharged());*/
  }

  private static boolean isBatteryCharged(){
    return RobotController.getBatteryVoltage() >= Constants.kMinimumBatteryVoltage;
  }

  private static boolean isCanConnected(){
    return DeviceFinder.Find().size() == Constants.kCanDeviceCount;
  }
  

  // configures button bindings to controller
  private void configureButtonBindings() {
    Trigger dpadUp = new Trigger(() -> {return m_controller.getDpadUp();});
    Trigger dpadUpRight = new Trigger(() -> {return m_controller.getDpadUpRight();});
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});
    Trigger dpadLeft = new Trigger(() -> {return m_controller.getDpadLeft();});
    Trigger dpadRight = new Trigger(() -> {return m_controller.getDpadRight();});

    PathPlannerTrajectory path = PathPlanner.loadPath("2 arcs", new PathConstraints(4, 3));
    dpadLeft.whenActive(DrivetrainSubsystem.getInstance().followTrajectoryCommand(path, false));

    final double triggerDeadzone = 0.2;
    m_controller.getStartButton().whenPressed(new InstantCommand(m_drivetrainSubsystem::resetOdometry));
    m_controller.getBackButton().whenPressed(new InstantCommand(m_drivetrainSubsystem::resetOdometry));
    
  }

  public void onRobotDisabled() {
    //called when robot is disabled. Set all subsytems to 0

  }

  //right trigger shoot
  //left trigger intake deploy and intake, release, fold back
  //left bumper eject
  //right bumper shooter while moving
  
}
