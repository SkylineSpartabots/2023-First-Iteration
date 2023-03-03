package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetMechanism;
import frc.robot.subsystems.CompleteMechanism.MechanismState;

public class Intake extends SubsystemBase {
    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null)
            instance = new Intake();
        return instance;
    }

    // change IDs
    private Solenoid intakePositionSolenoid, barSolenoid;
    private Compressor compressor;
    public IntakeStates intakeState = IntakeStates.OFF_DEPLOYED;
    private WPI_TalonFX mIntakeMotor;

    private Intake() {
        intakePositionSolenoid = new Solenoid(
                Constants.HardwarePorts.pneumaticHub,
                PneumaticsModuleType.REVPH,
                Constants.HardwarePorts.intakePositionSolenoidChannel);
        barSolenoid = new Solenoid(
            Constants.HardwarePorts.pneumaticHub,
            PneumaticsModuleType.REVPH,
            Constants.HardwarePorts.intakeBarSolenoidChannel);
        compressor = new Compressor(Constants.HardwarePorts.pneumaticHub, PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        mIntakeMotor = new WPI_TalonFX(Constants.HardwarePorts.intakeMotor);
        configureMotor(mIntakeMotor, false);
        setState(IntakeStates.OFF_RETRACTED);
    }

    private void configureMotor(WPI_TalonFX talon, boolean inverted) {
        talon.setInverted(inverted);
        talon.configVoltageCompSaturation(12.0, Constants.timeOutMs);
        talon.enableVoltageCompensation(true);
        talon.setNeutralMode(NeutralMode.Brake);
        talon.config_kF(0, 0.05, Constants.timeOutMs);
        talon.config_kP(0, 0.30, Constants.timeOutMs);
        talon.config_kI(0, 0, Constants.timeOutMs);
        talon.config_kD(0, 0, Constants.timeOutMs);
    }

    public enum IntakeStates {
        OFF_RETRACTED(false, false, 0),
        OFF_RETRACTED_CUBE(false, true, 0),
        ON_RETRACTED(false, false, -1),
        ON_RETRACTED_CUBE(false, true, 1),
        REV_RETRACTED(false, false, 1),
        OFF_DEPLOYED(true, false, 0),
        OFF_DEPLOYED_CUBE(true, true, 0),
        ON_DEPLOYED(true, false, -1),
        ON_DEPLOYED_CUBE(true, true, 1),
        REV_DEPLOYED(true, false, 1),
        REV_DEPLOYED_CUBE(true, true, -1),
        ON_DEPLOYED_GCONE(true, false, 1.2),
        OFF_DEPLOYED_GCONE(true, false, 0.075),
        OFF_RETRACTED_GCONE(false, false, 0.075);


        public boolean deployed;
        public boolean cube;
        public double direction;

        private IntakeStates(boolean deployed, boolean cube, double direction) {
            this.deployed = deployed;
            this.cube = cube;
            this.direction = direction;
        }
    }

    public void setState(IntakeStates state) {
        this.intakeState = state;
        intakePositionSolenoid.set(intakeState.deployed);
        barSolenoid.set(intakeState.cube);
        final double offset = 0.60;
        mIntakeMotor.set(ControlMode.PercentOutput, offset * intakeState.direction);
    }

    public void deployCommand() {
        CommandScheduler.getInstance().schedule(Intake.getInstance().intakeState.cube ? new SetIntake(IntakeStates.OFF_DEPLOYED_CUBE) : new SetIntake(IntakeStates.OFF_DEPLOYED));
    }

    public void testSolenoid(boolean b) {
        intakePositionSolenoid.set(b);
    }

    public boolean getIntakeDeployed() {
        return intakeState.deployed;
    }
    
    public boolean getIntakeCube() {
        return intakeState.cube;
    }

    private double coneThreshold = 53.5;
    public boolean hasCone(){
        double currentVolt = mIntakeMotor.getStatorCurrent();
        return currentVolt > coneThreshold;
    }

    public double cubeThreshold = 39;
    public boolean hasCube(){
        double currentVolt = mIntakeMotor.getStatorCurrent();
        return currentVolt > cubeThreshold;
    }

    private double gConeThreshold = 55;
    public boolean hasgCone(){
        double currentVolt = mIntakeMotor.getStatorCurrent();
        return currentVolt > gConeThreshold;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake current", mIntakeMotor.getStatorCurrent());
        SmartDashboard.putBoolean("intake deployed", getIntakeDeployed());
        SmartDashboard.putBoolean("intake cube", getIntakeCube());
        if (intakeState == IntakeStates.ON_DEPLOYED_CUBE) {
            if (hasCube()) {
                CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED_CUBE));
            }
        }
        if (intakeState == IntakeStates.ON_DEPLOYED) {
            if (hasCone()) {
                CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED));
            }
        }
        if (intakeState == IntakeStates.ON_RETRACTED) {
            if (hasCone()) {
                CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED));
            }
        }
        if (intakeState == IntakeStates.ON_RETRACTED_CUBE) {
            if (hasCube()) {
                CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED_CUBE));
            }
        }

        if (intakeState == IntakeStates.ON_DEPLOYED_GCONE) {
            if (hasgCone()) {
                CommandScheduler.getInstance().schedule(new WaitCommand(1.6).andThen(new SetIntake(IntakeStates.OFF_DEPLOYED_GCONE)));
            }
        }
        // if (intakeState == IntakeStates.REV_DEPLOYED) {
        //     if (!hasGamePiece()) {
        //         CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_DEPLOYED));
        //     }
        // }
        // if (intakeState == IntakeStates.REV_RETRACTED) {
        //     if (!hasGamePiece()) {
        //         CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED));
        //     }
        // }
    }

}
