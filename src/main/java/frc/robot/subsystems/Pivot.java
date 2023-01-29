package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
	private static Pivot instance;

	public static Pivot getInstance() {
		if (instance == null) {
			instance = new Pivot();
		}
		return instance;
	}

	private TalonFX mLeaderPivotMotor, mFollowerPivotMotor;
	private double velocity;
	private PivotStates pivotState = PivotStates.ZERO;
	
	public enum PivotStates {
		ZERO(0.0),
		GROUND(0.0),
		SUBSTATION(0.0),
		L1(0.0),
		L2(0.0),
		L3(0.0),
		TEST(1.0); // for testing

		double statePosition = 0.0;

		private PivotStates(double statePosition) {
			this.statePosition = statePosition;
		}
	}

	public Pivot() {
		mLeaderPivotMotor = new TalonFX(Constants.HardwarePorts.pivotLeaderMotor);
		configureMotor(mLeaderPivotMotor, false);
		mFollowerPivotMotor = new TalonFX(Constants.HardwarePorts.pivotFollowerMotor);
		configureMotor(mFollowerPivotMotor, true); 
		mFollowerPivotMotor.set(ControlMode.Follower, Constants.HardwarePorts.pivotLeaderMotor);
		setEncoderPosition(0);
	}

	private void configureMotor(TalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.config_kF(0, 0.05, Constants.timeOutMs);
        talon.config_kP(0, 0.12, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }

	public void setVelocity (double velocity) {
		this.velocity = velocity;
		mLeaderPivotMotor.set(ControlMode.Velocity, velocity);
	}

	public void setPosition (PivotStates state) {
		pivotState = state;
		mLeaderPivotMotor.set(ControlMode.Position, pivotState.statePosition);
	}

	private double position = 0;
    public void testPosition(boolean forward){
        position += forward ? 1000 : -1000;
        // mLeaderPivotMotor.set(ControlMode.Position, position);
    }
    public void testPosition(double pos){
        position = pos;
        mLeaderPivotMotor.set(ControlMode.Position, position);
    }

	public double getVelocitySetpoint () {
		return velocity;
	}

	public double getPositionSetpoint () {
		return pivotState.statePosition;
	}

	public double getMeasuredPosition () {
		return mLeaderPivotMotor.getSelectedSensorPosition();
	}

	public void setEncoderPosition (double position) {
		mLeaderPivotMotor.setSelectedSensorPosition(position);
		mFollowerPivotMotor.setSelectedSensorPosition(position);
	}

	@Override
	public void periodic() {
		// SmartDashboard.putNumber("pivot pos setpoint", getPositionSetpoint());
		// SmartDashboard.putNumber("pivot pos measured", getMeasuredPosition());
		// SmartDashboard.putNumber("pivot set velo", getVelocitySetpoint());
		SmartDashboard.putNumber("leader pos", mLeaderPivotMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("follower pos", mFollowerPivotMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("piv pos setpoint", position);
	}
}