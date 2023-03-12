package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.SetIntake;

public class Intake extends SubsystemBase {
    
    private static Intake instance;
    CANSparkMax m_leaderMotor, m_followerMotor;
    Solenoid m_solenoid;
    Compressor m_compressor;
    public IntakeStates intakeState = IntakeStates.OFF_CLOSED_CONE;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    Intake() {
        m_solenoid = new Solenoid(
            Constants.HardwarePorts.pneumaticHub,
            PneumaticsModuleType.REVPH,
            Constants.HardwarePorts.intakeSolenoidChannel);
        m_compressor = new Compressor(Constants.HardwarePorts.pneumaticHub, PneumaticsModuleType.REVPH);
        m_compressor.enableDigital();
        m_leaderMotor = new CANSparkMax(Constants.HardwarePorts.intakeLeaderMotor, MotorType.kBrushless);
        m_followerMotor = new CANSparkMax(Constants.HardwarePorts.intakeFollowerMotor, MotorType.kBrushless);
        m_followerMotor.follow(m_leaderMotor, true);
    }

    public enum IntakeStates {
        ON_CLOSED_CONE(false, "cone", 1), //spinning so we can intake, and then it is closed
        OFF_CLOSED_CONE(false, "cone", 0), 
        REV_CLOSED_CONE(false, "cone", -1),
        OFF_OPEN_CONE(true, "cone", 0),

        ON_OPEN_CUBE(true, "cube", 1),
        OFF_OPEN_CUBE(true, "cube", 0),
        REV_OPEN_CUBE(true, "cube", -1);

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
        m_solenoid.set(intakeState.deployed);
        final double offset = 0.75;
        setSpeed(offset * intakeState.direction);
    }

    public void setSpeed(double speed) {
        // DO NOT RUN THE INTAKE MOTOR RIGHT NOW
        // m_leaderMotor.set(speed);
    }

    public boolean getIntakeDeployed() {
        return intakeState.deployed;
    }
    
    public String getIntakePiece() {
        return intakeState.piece;
    }

    private double coneThreshold = 46.5;
    public boolean hasCone(){
        double currentVolt = m_leaderMotor.getOutputCurrent();
        return currentVolt > coneThreshold;
    }

    public double cubeThreshold = 39;
    public boolean hasCube(){
        double currentVolt = m_leaderMotor.getOutputCurrent();
        return currentVolt > cubeThreshold;
    }

    private double gConeThreshold = 55;
    public boolean hasLayedCone(){
        double currentVolt = m_leaderMotor.getOutputCurrent();
        return currentVolt > gConeThreshold;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("intake piece", getIntakePiece());
        // SmartDashboard.putNumber("intake current", leaderMotor.getStatorCurrent());
        // SmartDashboard.putBoolean("intake deployed", getIntakeDeployed());

        if (intakeState == IntakeStates.ON_OPEN_CUBE) {
            if (hasCube()) {
                CommandScheduler.getInstance().schedule(new WaitCommand(1.0).andThen(new SetIntake(IntakeStates.OFF_OPEN_CUBE)));
            }
        }

        if (intakeState == IntakeStates.ON_CLOSED_CONE) {
            if (hasCone()) {
                CommandScheduler.getInstance().schedule(new WaitCommand(1.0).andThen(new SetIntake(IntakeStates.OFF_CLOSED_CONE)));
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
