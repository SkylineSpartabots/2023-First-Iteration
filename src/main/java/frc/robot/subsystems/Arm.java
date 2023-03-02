package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

// import edu.wpi.first.networktables.NetworkMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    static Arm instance;

    public static Arm getInstance() {
        if (instance == null) instance = new Arm();
        return instance;
    }

    private WPI_TalonFX mArmMotor;
    private double velocity;
    private double voltage;
    private CANCoder armCANCoder = new CANCoder(Constants.HardwarePorts.armCANCoder); // max 420
    CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
    private ArmStates armState = ArmStates.ZERO;

    public enum ArmStates {
        ZERO(52.0), //when curled up
        GROUNDCONE(272), // 210 //intaking cone from ground MAY NEED TO CHANGE
        GROUNDCUBE(215), //intaking cube from ground
        SUBSTATION(150), //not measured yet
        L1CONE(150), 
        L2CONE(90), //middle scoring thing
        L3CONE(143), //upper scoring thing - not measured yet
        L1CUBE(55), 
        L2CUBE(111), //middle scoring thing
        L3CUBE(151),
        LAYEDCONE(271),
        TEST(220);

        double statePosition = 0.0;

        private ArmStates(double statePosition) {
            this.statePosition = statePosition;
        }

    }

    public Arm() {
        mArmMotor = new WPI_TalonFX(Constants.HardwarePorts.armMotor);
        configureMotor(mArmMotor, true);
        mArmMotor.setSelectedSensorPosition(0);
        canCoderConfig.sensorDirection = true;
        armCANCoder.configAllSettings(canCoderConfig);
        armCANCoder.setPosition(0);
    }

    private void configureMotor(WPI_TalonFX talon, boolean inverted){
        talon.setInverted(inverted);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(false);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kF(0, 0.05, Constants.timeOutMs);
        talon.config_kP(0, 0.12, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
        mArmMotor.set(ControlMode.Velocity, velocity);
    }
// ele 499 intake 271
    public void setVoltage(double voltage) {
        this.voltage = voltage;
        mArmMotor.setVoltage(voltage);
    }

    public void setState(ArmStates state) {
        armState = state;
    }

    public void setMotorPosition(double position) {
        mArmMotor.setSelectedSensorPosition(position);
    }

    public double getVelocitySetpoint() {
        return velocity;
    }
    
    public double getVoltageSetpoint() {
        return voltage;
    }

    public double getCANCoderSetpoint() {
        return armState.statePosition;
    }

    public double getMotorPosition () {
		return mArmMotor.getSelectedSensorPosition();
	}

    public void setCANCoderPosition(double position) {
        armCANCoder.setPosition(position);
    }

    public double getCANCoderPosition() {
        return (armCANCoder.getAbsolutePosition() - 283 + 360) % 360;
    }

    public double getCANCoderVoltage() {
        return armCANCoder.getBusVoltage();
    }

    public boolean armError() {
        return armCANCoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED;
    }
    
    private boolean inCoast = false;
    public void toggleNeutral(){
        inCoast = !inCoast;
        NeutralMode newNeutral = inCoast ? NeutralMode.Coast : NeutralMode.Brake;
        mArmMotor.setNeutralMode(newNeutral);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armCANpos", getCANCoderPosition());
		SmartDashboard.putNumber("armPosSet", getCANCoderSetpoint());
		// SmartDashboard.putNumber("arm set velo", getVelocitySetpoint());
		SmartDashboard.putNumber("arm set volt", getVoltageSetpoint());
        SmartDashboard.putNumber("arm CANCoder Voltage", getCANCoderVoltage());
        // SmartDashboard.putNumber("armMotpos", getMotorPosition());
        SmartDashboard.putBoolean("arm error", armError());
    }
}

