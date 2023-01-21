package frc.robot.subsystems;

import java.security.cert.Extension;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;

public class ExtensionSubsystem {
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
        extensionMotor.setNeutralMode(NeutralMode.Brake);
        extensionMotor.config_kP(0, 1.0);
        extensionMotor.config_kI(0, 0);
        extensionMotor.config_kD(0, 0);
    }
    
    public void setExtensionPosition(ExtensionControl position) {
        extensionMotor.set(ControlMode.Position, position.value);
    }
    public enum ExtensionControl {
        GROUND(9000),
        SUBSTATION(6000),
        FIRST_STAIR(4000),
        SECOND_STAIR(6000),
        THIRD_STAIR(9000);

        private int value;
        private ExtensionControl(int value) {
            this.value = value;
        }
    }
}
