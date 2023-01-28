package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

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
   
    final double extend = -110946;
    private boolean executablePosition;

    public enum ExtensionStates {
        ZERO(0.0),
        GROUND(0.0),
        SUBSTATION(0.0),
        L1(0.0), 
        L2(0.0),
        L3(0.0),
        TEST(1.0); // for testing

        double statePosition = 0.0;

        private ExtensionStates(double statePosition) {
            this.statePosition = statePosition;
        }
    }

    public Extension() {
        mExtensionMotor = new TalonFX(Constants.HardwarePorts.extensionMotor);
        configureMotor(mExtensionMotor, true);
        setEncoderPosition(0);
        position = getMeasuredPosition();
    }

    private void configureMotor(TalonFX talon, boolean b) {
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.config_kF(0, 0, Constants.timeOutMs);
        talon.config_kP(0, 0.1, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0.001, Constants.timeOutMs);
    }
    
    public void setVelocity(double velocity) {
        this.velocity = velocity;
        mExtensionMotor.set(ControlMode.Velocity, velocity);
    }

    public void setPosition(ExtensionStates state) {
        extensionState = state;
        mExtensionMotor.set(ControlMode.Position, state.statePosition);
        
    }

    private boolean invalidPosition(double pos){
        executablePosition = !(pos > Constants.ExtensionConstants.backEndPosition || pos < Constants.ExtensionConstants.frontEndPosition);
        return !executablePosition; //boolean junk
    }

    private double position = 0;
    public void testPosition(boolean forward){
        position += forward ? 5000 : -5000;
        mExtensionMotor.set(ControlMode.Position, position);
    }
    public void testPosition(double pos){
        position = pos;
        mExtensionMotor.set(ControlMode.Position, position);
    }

    public double getVelocitySetpoint () {
		return velocity;
	}

	public double getPositionSetpoint () {
		return extensionState.statePosition;
	}

	public double getMeasuredPosition () {
		return mExtensionMotor.getSelectedSensorPosition();
	}

    public void setEncoderPosition (double position) {
       mExtensionMotor.setSelectedSensorPosition(position);
	}

    public void resetEncoder(){
        // mExtensionMotor.resetEncoder();
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Valid Extension Position", executablePosition);
        SmartDashboard.putNumber("exten pos testpoint", position);
        SmartDashboard.putNumber("exten pos setpoint", getPositionSetpoint());
		SmartDashboard.putNumber("exten pos measured", getMeasuredPosition());
		SmartDashboard.putNumber("exten set velo", getVelocitySetpoint());
    }
}
