package frc.robot.subsystems;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.fasterxml.jackson.databind.jsontype.PolymorphicTypeValidator.Validity;

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
    private double current;
    private IntakeState state;
    private TalonFX mIntakemotor;

    private Intake() {
        solenoid = new DoubleSolenoid(
            PneumaticsModuleType.REVPH, 
            Constants.HardwarePorts.intakeSolenoidForwardChannel, 
            Constants.HardwarePorts.intakeSolenoidReverseChannel
        );
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        current = compressor.getPressure();
        solenoid.set(Value.kOff);
        mIntakemotor = new TalonFX(Constants.HardwarePorts.intakeMotor);
        configureMotor(mIntakemotor, false); // figure out inversion
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

    public enum IntakeState {
        OFF_RETRACTED(Value.kReverse, 0),
        ON_RETRACTED(Value.kReverse, 1),
        REV_RETRACTED(Value.kReverse, -1),
        OFF_DEPLOYED(Value.kForward, 0),
        ON_DEPLOYED(Value.kForward, 1),
        REV_DEPLOYED(Value.kForward, -1);

        Value value;
        int direction;

        private IntakeState(Value value, int direction) {
            this.value = value;
            this.direction = direction;
        }
    }    

    public void setState(IntakeState state) {
        solenoid.set(state.value);
        final int offset = 8000;
        mIntakemotor.set(TalonFXControlMode.Velocity, offset * state.direction);
        this.state = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("intake motor value", state.value == Value.kForward ? "forward" : "reverse");
        SmartDashboard.putNumber("intake motor direction", state.direction);
    }
}
