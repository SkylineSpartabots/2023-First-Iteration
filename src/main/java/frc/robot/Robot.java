// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.*;

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
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    RobotContainer.printDiagnostics();
    CameraServer.startAutomaticCapture();
  }

  @Override
  public void robotPeriodic() {
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

  @Override
  public void autonomousInit() {

    // schedule the autonomous command
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
    if (m_autonomousCommand != null) {
      // cancels auto command
      m_autonomousCommand.cancel();
    }
    //new RobotOff().schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    

  }

  @Override
  public void testInit() {
    
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    //new DiagnosticTestCommand().schedule();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
