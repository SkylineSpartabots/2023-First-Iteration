package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    static ElevatorSubsystem instance;
    public ElevatorSubsystem getInstance() {
        if (instance == null) instance = new ElevatorSubsystem();
        return instance;
    }
    private TalonFX mLeaderElevatorMotor, mFollowerElevatorMotor;
    private double velocity;
    ElevatorStates elevatorState = ElevatorStates.ZERO;

    public enum ElevatorStates {
		ZERO(0.0),
		GROUND(0.0),
		SUBSTATION(0.0),
		L1(0.0),
		L2(0.0),
		L3(0.0),
		TEST(1.0); // for testing

		double statePosition = 0.0;

		private ElevatorStates(double statePosition) {
			this.statePosition = statePosition;
		}
	}
    
    public ElevatorSubsystem() {
        mLeaderElevatorMotor = new TalonFX(Constants.HardwarePorts.elevatorLeaderMotor);
        configureMotor(mLeaderElevatorMotor, false);
        mFollowerElevatorMotor = new TalonFX(Constants.HardwarePorts.elevatorFollowerMotor, "2976 CANivore");
        configureMotor(mFollowerElevatorMotor, false);
        mFollowerElevatorMotor.set(ControlMode.Follower, Constants.HardwarePorts.elevatorLeaderMotor);
    }

    private void configureMotor(TalonFX talon, boolean b){
        talon.setInverted(b);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kF(0, 0, Constants.timeOutMs);
        talon.config_kP(0, 0, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }

    public setVelocity(double velocity) {
        this.velocity = velocity;
        mLeaderElevatorMotor.set(ControlMode.Velocity, velocity);
    }
    public setPosition(ElevatorStates state) {
        elevatorState = state;
        mLeaderElevatorMotor.set(ControlMode.Position, state);
    }
    public getVelocitySetpoint() {
        return velocity;
    }
    public getPositionSetpoint() {
        return elevatorState.statePosition;
    }
    public double getMeasuredPosition () {
		return mLeaderElevatorMotor.getSelectedSensorPosition();
	}

	public void setEncoderPosition (double position) {
		mLeaderElevatorMotor.setSelectedSensorPosition(position);
	}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator pos setpoint", getPositionSetpoint());
		SmartDashboard.putNumber("elevator pos measured", getMeasuredPosition());
		SmartDashboard.putNumber("elevator set velo", getVelocitySetpoint());
    }
}
