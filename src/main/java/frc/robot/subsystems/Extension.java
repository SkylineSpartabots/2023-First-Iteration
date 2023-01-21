package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Extension extends SubsystemBase {
    static Extension instance;

    public static Extension getInstance() {
        if (instance == null) {
            instance = new Extension();
        }
        return instance;
    }
     
    private TalonFX mExtensionMotor;
    private double velocity;
    private ExtensionStates extensionState = ExtensionStates.ZERO;
    private double position = extensionState.statePosition;
    
    public enum ExtensionStates {
        ZERO(0.0),
        GROUND(0.0),
        SUBSTATION(0.0),
        L1(0.0),
        L2(0.0),
        L3(0.0);

        double statePosition = 0.0;

        private ExtensionStates(double statePosition) {
            this.statePosition = statePosition;
        }
    }

    public Extension() {
        mExtensionMotor = new TalonFX(Constants.HardwarePorts.extensionMotor);
        configureMotor(mExtensionMotor, false); // check inversion
    }

    private void configureMotor(TalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kF(0, 0.05, Constants.timeOutMs);
        talon.config_kP(0, 0.12, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }
    
    public void setExtensionVelocity(double velocity) {
        this.velocity = velocity;
        mExtensionMotor.set(ControlMode.Velocity, velocity);
    }

    public void setExtensionPosition(ExtensionStates extensionState) {
        this.extensionState = extensionState;
        mExtensionMotor.set(ControlMode.Position, position);
    }

    public double getVelocitySetpoint () {
		return velocity;
	}

	public double getPositionSetpoint () {
		return position;
	}

	public double getMeasuredPosition () {
		return mExtensionMotor.getSelectedSensorPosition();
	}

    public void setEncoderPosition (double position) {
		mExtensionMotor.setSelectedSensorPosition(position);
	}
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("exten pos setpoint", getPositionSetpoint());
		SmartDashboard.putNumber("exten pos measured", getMeasuredPosition());
		SmartDashboard.putNumber("exten set velo", getVelocitySetpoint());
    }
}
