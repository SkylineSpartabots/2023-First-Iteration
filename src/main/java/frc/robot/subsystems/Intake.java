package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) 
            instance = new Intake();
        return instance;
    } 

    // change IDs
    private DoubleSolenoid solenoid;
    private Compressor compressor;
    private IntakeStates intakeState;
    private TalonFX mIntakemotor;

    private Intake() {
        solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            Constants.HardwarePorts.intakeSolenoidForwardChannel, 
            Constants.HardwarePorts.intakeSolenoidReverseChannel
        );
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        mIntakemotor = new TalonFX(Constants.HardwarePorts.intakeMotor, "2976 CANivore");
        configureMotor(mIntakemotor, false); // figure out inversion
        setState(IntakeStates.OFF);
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

    public enum IntakeStates {
        OFF(Value.kOff, 0),
        OFF_RETRACTED(Value.kReverse, 0),
        ON_RETRACTED(Value.kReverse, 1),
        REV_RETRACTED(Value.kReverse, -1),
        OFF_DEPLOYED(Value.kForward, 0),
        ON_DEPLOYED(Value.kForward, 1),
        REV_DEPLOYED(Value.kForward, -1);

        Value value;
        int direction;

        private IntakeStates(Value value, int direction) {
            this.value = value;
            this.direction = direction;
        }
    }    

    public void setState(IntakeStates state) {
        this.intakeState = state;
        solenoid.set(intakeState.value);
        final int offset = 8000;
        mIntakemotor.set(TalonFXControlMode.Velocity, offset * intakeState.direction);
    }
    
    public void testVelo(int direction) {
        mIntakemotor.set(TalonFXControlMode.Velocity, 8000 * direction);
    }

    public int getVelocityDirection() {
        return intakeState.direction;
    }

    public boolean getIntakeDeployed() {
        return intakeState.value == Value.kForward ? true : false;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake motor deployed", getIntakeDeployed());
        SmartDashboard.putNumber("intake motor direction", getVelocityDirection());
    }
}
