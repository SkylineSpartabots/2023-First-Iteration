package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
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
    private Solenoid intakeSolenoid;
    private Compressor compressor;
    private IntakeStates intakeState = IntakeStates.OFF_DEPLOYED;
    private TalonFX mIntakeMotor;

    private Intake() {
        intakeSolenoid = new Solenoid(
            Constants.HardwarePorts.pneumaticHub,
            PneumaticsModuleType.REVPH, 
            Constants.HardwarePorts.intakeSolenoidChannel
        );
        compressor = new Compressor(Constants.HardwarePorts.pneumaticHub, PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        mIntakeMotor = new TalonFX(Constants.HardwarePorts.intakeMotor);
        configureMotor(mIntakeMotor, false); 
        setState(IntakeStates.OFF_RETRACTED);
    }

    private void configureMotor(TalonFX talon, boolean inverted){
        talon.setInverted(inverted);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kF(0, 0.05, Constants.timeOutMs);
        talon.config_kP(0, 0.12, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }

    public enum IntakeStates {
        OFF_RETRACTED(false, 0),
        ON_RETRACTED(false, 1),
        REV_RETRACTED(false, -1),
        OFF_DEPLOYED(true, 0),
        ON_DEPLOYED(true, 1),
        REV_DEPLOYED(true, -1);

        boolean value;
        int direction;

        private IntakeStates(boolean value, int direction) {
            this.value = value;
            this.direction = direction;
        }
    } 

    public void setState(IntakeStates state) {
        this.intakeState = state;
        intakeSolenoid.set(intakeState.value);
        final int offset = 8000;
        mIntakeMotor.set(ControlMode.Velocity, offset * intakeState.direction);
    }

    public void testVelo(int direction) {
        mIntakeMotor.set(ControlMode.Velocity, 8000 * direction);
    }

    public void testSolenoid(boolean b) {
        intakeSolenoid.set(b);
    }

    public int getVelocityDirection() {
        return intakeState.direction;
    }

    public boolean getIntakeDeployed() {
        return intakeState.value;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake motor deployed", getIntakeDeployed());
        SmartDashboard.putNumber("intake motor direction", getVelocityDirection());
    }
}
