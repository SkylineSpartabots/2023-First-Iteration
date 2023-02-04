package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    static Elevator instance;
    public static Elevator getInstance() {
        if (instance == null) instance = new Elevator();
        return instance;
    }

    private TalonFX mLeaderElevatorMotor, mFollowerElevatorMotor;
    private double velocity;
    private CANCoder elevatorCANCoder = new CANCoder(Constants.HardwarePorts.elevatorCANCoder);
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
    
    double positionSetpoint = 0; 
    
    public Elevator() {
        mLeaderElevatorMotor = new TalonFX(Constants.HardwarePorts.elevatorLeaderMotor);
        configureMotor(mLeaderElevatorMotor, false);
        mFollowerElevatorMotor = new TalonFX(Constants.HardwarePorts.elevatorFollowerMotor);
        configureMotor(mFollowerElevatorMotor, false);
        mFollowerElevatorMotor.set(ControlMode.Follower, Constants.HardwarePorts.elevatorLeaderMotor);
        mLeaderElevatorMotor.setSelectedSensorPosition(0);
        mFollowerElevatorMotor.setSelectedSensorPosition(0);
    }

    private void configureMotor(TalonFX talon, boolean inverted){
        talon.setInverted(inverted);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Coast);
        talon.config_kF(0, 0.05, Constants.timeOutMs);
        talon.config_kP(0, 0.12, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }

    public void changePosition(boolean up) {
        elevatorState.statePosition += up ? 100 : -100;
        mLeaderElevatorMotor.set(ControlMode.Position, elevatorState.statePosition);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        mLeaderElevatorMotor.set(ControlMode.Velocity, velocity);
    }

    public void setState(ElevatorStates state) {
        elevatorState = state;
        // mLeaderElevatorMotor.set(ControlMode.Position, state.statePosition);
    }

    public void setPosition(int position) {
        positionSetpoint = position;
        mLeaderElevatorMotor.set(ControlMode.Position, position);
    }

    public double getVelocitySetpoint() {
        return velocity;
    }

    public double getPositionSetpoint() {
        return elevatorState.statePosition;
    }

    public double getMeasuredPosition() {
		return mLeaderElevatorMotor.getSelectedSensorPosition();
	}

    public void setCANCoderPosition(double position) {
        elevatorCANCoder.setPosition(position);
    }

    public double getCANCoderPosition() {
        return elevatorCANCoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("leader motor pos measured", mLeaderElevatorMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("follower motor pos measured", mFollowerElevatorMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("elevator pos setpoint", getPositionSetpoint());
		SmartDashboard.putNumber("elevator pos measured", getMeasuredPosition());
		SmartDashboard.putNumber("elevator set velo", getVelocitySetpoint());
        SmartDashboard.putNumber("ESETPT", positionSetpoint);

    }
}
