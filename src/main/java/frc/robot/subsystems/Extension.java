package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Extension extends SubsystemBase {
    static Extension instance;

    public static Extension getInstance() {
        if (instance == null) {
            instance = new Extension();
        }
        return instance;
    }
     
    private CANSparkMax mExtensionMotor; 
    private SparkMaxPIDController mPIDController;
    private RelativeEncoder mEncoder;
    private double velocity;
    private ExtensionStates extensionState = ExtensionStates.ZERO;
    private double position = extensionState.statePosition;
    
    public enum ExtensionStates {
        ZERO(0.0),
        GROUND(0.0),
        SUBSTATION(0.0),
        L1(5.0), //testing purposes
        L2(0.0),
        L3(0.0);

        double statePosition = 0.0;

        private ExtensionStates(double statePosition) {
            this.statePosition = statePosition;
        }
    }

    public Extension() {
        mExtensionMotor = new CANSparkMax(20, MotorType.kBrushless);
        mExtensionMotor.restoreFactoryDefaults();
        mPIDController = mExtensionMotor.getPIDController();
        mEncoder = mExtensionMotor.getEncoder();

        configureMotor(); // check inversion
        setExtensionPosition(ExtensionStates.GROUND); // arbitrary, using it for testing
    }

    private void configureMotor(){
        mPIDController.setD(6e-5);
        mPIDController.setI(0);
        mPIDController.setP(0);
        mPIDController.setFF(0.000015);
        mEncoder.setInverted(false);
    }
    
    public void setExtensionVelocity(double velocity) {
        this.velocity = velocity;
        mPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }

    public void setExtensionPosition(ExtensionStates extensionState) {
        this.extensionState = extensionState;
        position = this.extensionState.statePosition;
        mPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
    }

    public double getVelocitySetpoint () {
		return velocity;
	}

	public double getPositionSetpoint () {
		return position;
	}

	public double getMeasuredPosition () {
		return mEncoder.getPosition();
	}

    public void setEncoderPosition (double position) {
        mPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
	}
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("exten pos setpoint", getPositionSetpoint());
		SmartDashboard.putNumber("exten pos measured", getMeasuredPosition());
		SmartDashboard.putNumber("exten set velo", getVelocitySetpoint());
    }
}
