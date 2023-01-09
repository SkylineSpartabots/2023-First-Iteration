// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LazyTalonFX;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  static ArmSubsystem instance = null;
    public static ArmSubsystem getInstance(){
        if(instance == null){
            instance = new ArmSubsystem();
        }
        return instance;
    }
    static LazyTalonFX m_ArmBaseMotor;
    static LazyTalonFX m_ArmJointMotor;
  public ArmSubsystem() {
    m_ArmBaseMotor = new LazyTalonFX("base", 0); //TODO
    m_ArmJointMotor = new LazyTalonFX("joint", 0); //TODO
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Middle Joint Position", m_ArmJointMotor.getActiveTrajectoryPosition());
    SmartDashboard.putNumber("Base Joint Position", m_ArmBaseMotor.getActiveTrajectoryPosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

//Claw grip strength is handled in pnuematics. no difference in arm movement when picking up a cube vs cone
    enum extensionDistance {
        IN (0.00),
        OUT (1.0);

        public double distance = 0.0;

        private extensionDistance(double distance){
            this.distance = distance;


        }
    }
}

