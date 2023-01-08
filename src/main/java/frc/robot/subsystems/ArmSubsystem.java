// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    public static ArmSubsystem getInstance(){
        if(instance == null){
            instance = new ArmSubsystem();
        }
        return instance;
    }
  
  public ArmSubsystem() {
    m_ArmBaseMotor =  //TODO
    m_ArmJointMotor = //TODO
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Middle Joint Pitch", m_ArmJointMotor.getPitch);
    SmartDashboard.putNumber("Base Joint Pitch", m_ArmBaseMotor.getPitch);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

//Claw grip strength is handled in pnuematics. no difference in arm movement when picking up a cube vs cone 
  

  public void setBasePitch(){}

  public void setBaseYaw(){}

  public void setExtensionDistance(){}

  public double getBasePitch(){
    return basePitch;
  }

  public double getBaseYaw(){
    return jointYaw;
  }

  public double getArmExtension(){
    return extensionLength;
  }
}

enum extensionDistance {
    IN (0.00),
    OUT (1.0);

    public double distance = 0.0;

    private extensionDistance(double distance){
        this.distance = distance;


    }
}
public void setElevator(State state) {
    extensionMotor.set(//TODO);
}

