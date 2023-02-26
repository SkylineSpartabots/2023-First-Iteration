package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.AnalogEncoder;
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
    AnalogEncoder lamprey = new AnalogEncoder(Constants.HardwarePorts.armLamprey);
    private ArmStates armState = ArmStates.ZERO;

    public enum ArmStates {
        ZERO(10.0), //when curled up
        GROUNDCONE(155), //intaking cone from ground
        GROUNDCUBE(165), //intaking cube from ground
        SUBSTATION(150), //not measured yet
        L1CONE(150), 
        L2CONE(92.0), //middle scoring thing
        L3CONE(0.0), //upper scoring thing - not measured yet
        L1CUBE(150), 
        L2CUBE(121.0), //middle scoring thing
        L3CUBE(129.0),
        TEST(50);

        double statePosition = 0.0;

        private ArmStates(double statePosition) {
            this.statePosition = statePosition;
        }

    }

    public Arm() {
        mArmMotor = new WPI_TalonFX(Constants.HardwarePorts.armMotor);
        configureMotor(mArmMotor, true);
        mArmMotor.setSelectedSensorPosition(0);
        lamprey.reset();
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

    public double getLampreySetpoint() {
        return armState.statePosition;
    }

    public double getMotorPosition () {
		return mArmMotor.getSelectedSensorPosition();
	}

    public void setLampreuyPosition(double position) {
        lamprey.setPositionOffset(position);
    }

    public double getLampreyPosition() {
        return lamprey.get();
    }

    public boolean armError() {
        return false;
    }
    
    private boolean inCoast = false;
    public void toggleNeutral(){
        inCoast = !inCoast;
        NeutralMode newNeutral = inCoast ? NeutralMode.Coast : NeutralMode.Brake;
        mArmMotor.setNeutralMode(newNeutral);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armCANpos", getLampreyPosition());
		SmartDashboard.putNumber("armPosSet", getLampreySetpoint());
		// SmartDashboard.putNumber("arm set velo", getVelocitySetpoint());
		SmartDashboard.putNumber("arm set volt", getVoltageSetpoint());
        // SmartDashboard.putNumber("armMotpos", getMotorPosition());
        SmartDashboard.putBoolean("arm error", armError());
    }
}


