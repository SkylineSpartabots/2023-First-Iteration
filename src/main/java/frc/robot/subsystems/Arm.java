package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    static Arm instance;

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    private TalonFX mArmMotor;
    private double velocity;
    private CANCoder armCANCoder = new CANCoder(Constants.HardwarePorts.armCANCoder);
    private ArmStates armState = ArmStates.ZERO;

    public enum ArmStates {
        ZERO(0.0),
        GROUND(0.0),
        SUBSTATION(0.0),
        L1(0.0), 
        L2(0.0),
        L3(0.0),
        TEST(0.5);

        double statePosition = 0.0;

        private ArmStates(double statePosition) {
            this.statePosition = statePosition;
        }

    }

    public Arm() {
        mArmMotor = new TalonFX(Constants.HardwarePorts.armMotor);
        configureMotor(mArmMotor, false);
    }

    private void configureMotor(TalonFX talon, boolean inverted){
        talon.setInverted(inverted);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.config_kF(0, 0.05, Constants.timeOutMs);
        talon.config_kP(0, 0.12, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }


    public void changePosition(boolean forward) {
        armState.statePosition += forward ? 3 : -3;
        mArmMotor.set(ControlMode.Position, armState.statePosition);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        mArmMotor.set(ControlMode.Velocity, velocity);
    }

    public void setState(ArmStates state) {
        armState = state;
        // mArmMotor.set(ControlMode.Position, state.statePosition);
    }

    public void setPosition(double position) {
        mArmMotor.setSelectedSensorPosition(position);
    }

    public double getVelocitySetpoint() {
        return velocity;
    }

    public double getPositionSetpoint() {
        return armState.statePosition;
    }

    public double getMeasuredPosition () {
		return mArmMotor.getSelectedSensorPosition();
	}

    public void setCANCoderPosition(double position) {
        armCANCoder.setPosition(position);
    }

    public double getCANCoderPosition() {
        return armCANCoder.getPosition();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm pos setpoint", getPositionSetpoint());
		SmartDashboard.putNumber("arm pos measured", getMeasuredPosition());
		SmartDashboard.putNumber("arm set velo", getVelocitySetpoint());
    }
}

