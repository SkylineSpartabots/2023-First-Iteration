package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.MagnetFieldStrength;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private static Elevator instance;

    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }

    private WPI_TalonFX mLeaderElevatorMotor, mFollowerElevatorMotor;
    private double velocity;
    private double voltage;
    private CANCoder elevatorCANCoder = new CANCoder(Constants.HardwarePorts.elevatorCANCoder); // max 2470
    CANCoderConfiguration canCoderConfig = new CANCoderConfiguration();
    ElevatorStates elevatorState = ElevatorStates.ZERO;

    public enum ElevatorStates { 
        ZERO(50),
        REALZERO(0),

        GROUNDINTAKE(50),
        SLEEPCONEINTAKE(2371),
        SLEEPCONEINTAKE2(2076),
        // LAYEDCONE(499),
        SUBSTATION(50), 
        DOUBLESUBSTATION(2080),

        L1CONE(50),
        L2CONE(1155),
        L3CONE(2463),

        L1CUBE(50),
        L2CUBE(454),
        L3CUBE(1926);

        // L1LAYEDCONE(0.0),
        // L2LAYEDCONE(1145),
        // L3LAYEDCONE(2256);

        double statePosition = 0.0;

        private ElevatorStates(double statePosition) {
            this.statePosition = statePosition;
        }
    }

    public Elevator() {
        mLeaderElevatorMotor = new WPI_TalonFX(Constants.HardwarePorts.elevatorLeaderMotor);
        configureMotor(mLeaderElevatorMotor, true);
        mFollowerElevatorMotor = new WPI_TalonFX(Constants.HardwarePorts.elevatorFollowerMotor);
        configureMotor(mFollowerElevatorMotor, true);
        mFollowerElevatorMotor.set(ControlMode.Follower, Constants.HardwarePorts.elevatorLeaderMotor);
        canCoderConfig.sensorDirection = false;
        elevatorCANCoder.configAllSettings(canCoderConfig);
        setCANCoderPosition(0);
    }

    private void configureMotor(WPI_TalonFX talon, boolean inverted) {
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
        mLeaderElevatorMotor.set(ControlMode.Velocity, velocity);
    }

    public void setVoltage(double voltage) {
        this.voltage = voltage;
        mLeaderElevatorMotor.setVoltage(voltage);
    }

    public void setState(ElevatorStates state) {
        elevatorState = state;
    }

    public double getVelocitySetpoint() {
        return velocity;
    }

    public double getVoltageSetpoint() {
        return voltage;
    }

    public double getCANCoderSetpoint() {
        return elevatorState.statePosition;
    }

    public void setCANCoderPosition(double position) {
        elevatorCANCoder.setPosition(position);
    }

    public double getCANCoderPosition() {
        return elevatorCANCoder.getPosition();
    }

    public double getCANCoderVoltage() {
        return elevatorCANCoder.getBusVoltage();
    }

    public boolean elevatorError() {
        if (elevatorCANCoder.getMagnetFieldStrength() == MagnetFieldStrength.BadRange_RedLED) {
            return true;
        }
        return false;
    }

    public double getCurrent() {
        return mLeaderElevatorMotor.getStatorCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("eleCANpos", getCANCoderPosition());
        SmartDashboard.putNumber("elePosSet", getCANCoderSetpoint());
        // SmartDashboard.putNumber("ele set velo", getVelocitySetpoint());
        // SmartDashboard.putNumber("ele set volt", getVoltageSetpoint());
        // SmartDashboard.putBoolean("elevator ", elevatorError());
        SmartDashboard.putNumber("eleCurrent", getCurrent());
    }
}
