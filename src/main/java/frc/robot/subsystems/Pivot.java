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

	private TalonFX mMasterPivotMotor, mFollowerPivotMotor;
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
		mMasterPivotMotor = new TalonFX(Constants.HardwarePorts.pivotMasterMotor);
		configureMotor(mMasterPivotMotor, false);
		mFollowerPivotMotor = new TalonFX(Constants.HardwarePorts.pivotFollowerMotor, "2976 CANivore");
		configureMotor(mFollowerPivotMotor, false); // check inversions for motors
		mFollowerPivotMotor.set(ControlMode.Follower, Constants.HardwarePorts.pivotMasterMotor);
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

	public void setVelocity (double velocity) {
		this.velocity = velocity;
		mMasterPivotMotor.set(ControlMode.Velocity, velocity);
	}

	public void setPosition (PivotStates state) {
		pivotState = state;
		mMasterPivotMotor.set(ControlMode.Position, pivotState.statePosition);
	}

	public double getVelocitySetpoint () {
		return velocity;
	}

	public double getPositionSetpoint () {
		return pivotState.statePosition;
	}

	public double getMeasuredPosition () {
		return mMasterPivotMotor.getSelectedSensorPosition();
	}

	public void setEncoderPosition (double position) {
		mMasterPivotMotor.setSelectedSensorPosition(position);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("pivot pos setpoint", getPositionSetpoint());
		SmartDashboard.putNumber("pivot pos measured", getMeasuredPosition());
		SmartDashboard.putNumber("pivot set velo", getVelocitySetpoint());
	}
}
