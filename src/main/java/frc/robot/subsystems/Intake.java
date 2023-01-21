package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) 
            instance = new Intake();
        return instance;
    } 

    // change IDs
    DoubleSolenoid solenoid;
    Compressor compressor;
    double current;
    IntakeState state;
    TalonFX m_intakemotor;

    private Intake() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 69, 69);
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        current = compressor.getPressure();
        solenoid.set(Value.kOff);
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
        m_intakemotor.set(TalonFXControlMode.Position, offset * state.direction);
        this.state = state;
    }

    @Override
    public void periodic() {

    }
}
