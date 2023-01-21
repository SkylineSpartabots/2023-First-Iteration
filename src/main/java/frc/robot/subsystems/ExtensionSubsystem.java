package frc.robot.subsystems;

import java.security.cert.Extension;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ExtensionSubsystem extends SubsystemBase {
    static ExtensionSubsystem instance = null;
    public static ExtensionSubsystem getInstance() {
        if (instance == null) {
            instance = new ExtensionSubsystem();
        }
        return instance;
    }
     
    TalonFX extensionMotor;
    

    public ExtensionSubsystem() {
        extensionMotor = new TalonFX(Constants.Arm.extensionMotorID);
        extensionMotor.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        extensionMotor.enableVoltageCompensation(true);
        extensionMotor.setNeutralMode(NeutralMode.Brake);
        extensionMotor.config_kP(0, 1.0);
        extensionMotor.config_kI(0, 0);
        extensionMotor.config_kD(0, 0);
    }
    
    public void setExtensionPosition(ExtensionControl position) {
        extensionMotor.set(ControlMode.Position, position.motorPosition);
    }
    public void setExtensionVelocity(double velocity) {
        extensionMotor.set(ControlMode.Velocity, velocity);
    }
    public enum ExtensionControl {
        GROUND(9000),
        SUBSTATION(6000),
        FIRST_STAIR(4000),
        SECOND_STAIR(6000),
        THIRD_STAIR(9000);

        double motorPosition;
        private ExtensionControl(int value) {
            this.motorPosition = value;
        }
    }
    
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("motor position")
    }
}
