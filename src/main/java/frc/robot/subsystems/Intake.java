package frc.robot.subsystems;

import javax.print.attribute.standard.Compression;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetIntake;

public class Intake extends SubsystemBase{
    private static Intake instance;

    private WPI_TalonFX leaderMotor, followerMotor;
    private Solenoid solenoid;
    public IntakeStates intakeState = IntakeStates.OFF_DEPLOYED_CONE;

    private Compressor compressor;

    public static Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    public Intake(){
        solenoid = new Solenoid(
            Constants.HardwarePorts.pneumaticHub,
            PneumaticsModuleType.REVPH,
            Constants.HardwarePorts.intakePositionSolenoidChannel);
        compressor = new Compressor(Constants.HardwarePorts.pneumaticHub, PneumaticsModuleType.REVPH);
        compressor.enableDigital();
        leaderMotor = new WPI_TalonFX(Constants.HardwarePorts.intakeMotor);
        followerMotor = new WPI_TalonFX(Constants.HardwarePorts.followerIntakeMotor);
        configureMotor(leaderMotor, false);
        configureMotor(followerMotor, false);
        followerMotor.set(ControlMode.Follower, Constants.HardwarePorts.intakeMotor); //inverse later
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
        ON_DEPLOYED_CONE(false, "cone", -1), //false
        OFF_DEPLOYED_CONE(false, "cone", 0), //when they put the intake down, doesnt need to change because it shouldn't change initial state
        REV_DEPLOYED_CONE(false, "cone", 1),
        OFF_RETRACTED_CONE(true, "cone", 0),

        ON_RETRACTED_CUBE(true, "cube", 1),
        OFF_RETRACTED_CUBE(true, "cube", 0),
        REV_RETRACTED_CUBE(true, "cube", -1);

        // ON_DEPLOYED_LAYEDCONE(true, "layed", 1.2),
        // OFF_DEPLOYED_LAYEDCONE(true, "layed", 0.075),
        // OFF_RETRACTED_LAYEDCONE(false, "layed", 0.075),
        // REV_RETRACTED_LAYEDCONE(false, "layed", -1.2);


        public boolean deployed;
        public String piece;
        public double direction;

        private IntakeStates(boolean deployed, String piece, double direction) {
            this.deployed = deployed;
            this.piece = piece;
            this.direction = direction;
        }
    }

    public void setState(IntakeStates state) {
        this.intakeState = state;
        solenoid.set(intakeState.deployed);
        final double offset = 0.75;
        leaderMotor.set(ControlMode.PercentOutput, offset * intakeState.direction);
    }

    public boolean getIntakeDeployed() {
        return intakeState.deployed;
    }
    
    public String getIntakePiece() {
        return intakeState.piece;
    }

    private double coneThreshold = 46.5;
    public boolean hasCone(){
        double currentVolt = leaderMotor.getStatorCurrent();
        return currentVolt > coneThreshold;
    }

    public double cubeThreshold = 39;
    public boolean hasCube(){
        double currentVolt = leaderMotor.getStatorCurrent();
        return currentVolt > cubeThreshold;
    }

    private double gConeThreshold = 55;
    public boolean hasLayedCone(){
        double currentVolt = leaderMotor.getStatorCurrent();
        return currentVolt > gConeThreshold;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("intake cube", getIntakePiece());
        // SmartDashboard.putNumber("Intake current", leaderMotor.getStatorCurrent());
        // SmartDashboard.putBoolean("intake deployed", getIntakeDeployed());

        if (intakeState == IntakeStates.ON_RETRACTED_CUBE) {
            if (hasCube()) {
                CommandScheduler.getInstance().schedule(new WaitCommand(1.0).andThen(new SetIntake(IntakeStates.OFF_RETRACTED_CUBE)));
            }
        }

        if (intakeState == IntakeStates.ON_DEPLOYED_CONE) {
            if (hasCone()) {
                CommandScheduler.getInstance().schedule(new WaitCommand(1.0).andThen(new SetIntake(IntakeStates.OFF_DEPLOYED_CONE)));
            }
        }
        
        // if (intakeState == IntakeStates.ON_DEPLOYED_LAYEDCONE) {
        //     if (hasLayedCone()) {
        //         CommandScheduler.getInstance().schedule(new WaitCommand(1.6).andThen(new SetIntake(IntakeStates.OFF_RETRACTED_LAYEDCONE)));
        //     }
        // }

        // if (intakeState == IntakeStates.ON_RETRACTED_CONE) {
        //     if (hasCone()) {
        //         CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED_CONE));
        //     }
        // }
        // if (intakeState == IntakeStates.ON_RETRACTED_CUBE) {
        //     if (hasCube()) {
        //         CommandScheduler.getInstance().schedule(new SetIntake(IntakeStates.OFF_RETRACTED_CUBE));
        //     }
        // }
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
