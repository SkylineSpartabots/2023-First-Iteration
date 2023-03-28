/*
 intake subsystem, encapsulates methods to control the intake mechanism
*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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
    private Swerve s_Swerve;
    CANSparkMax m_leaderMotor, m_followerMotor;
    Solenoid m_solenoid;
    Compressor m_compressor;
    public IntakeStates intakeState = IntakeStates.OFF_CLOSED_CONE;
    public BooleanSupplier motorStopped = () -> motorStopped();
    public BooleanSupplier shouldStopOnAuto = () -> stopMovingOnAuto();
    int cubeCounter;
    int coneCounter;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    // all states for the intake mechanism
    private final Light s_Lights = Light.getInstance();
    public enum IntakeStates {
        ON_CLOSED_CONE(false, "cone", 1.0),
        OFF_CLOSED_CONE(false, "cone", 0),
        REV_CLOSED_CONE(false, "cone", -1),
        OFF_OPEN_CONE(true, "cone", 0),

        ON_OPEN_CUBE(true, "cube", 0.75),
        OFF_CLOSED_CUBE(false, "cube", 0),
        REV_OPEN_CUBE(true, "cube", -0.5);

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

    // initializes hardward components, 2 motors, solenoid, and compressor
    // one of the motor in in follower moder
    Intake() {
        s_Swerve = Swerve.getInstance();
        cubeCounter = 0;
        coneCounter = 0;
        m_solenoid = new Solenoid(
                Constants.HardwarePorts.pneumaticHub,
                PneumaticsModuleType.REVPH,
                Constants.HardwarePorts.intakeSolenoidChannel);
        m_compressor = new Compressor(Constants.HardwarePorts.pneumaticHub, PneumaticsModuleType.REVPH);
        m_compressor.enableDigital();
        m_leaderMotor = new CANSparkMax(Constants.HardwarePorts.intakeLeaderMotor, MotorType.kBrushless);
        m_leaderMotor.setInverted(true);
        m_followerMotor = new CANSparkMax(Constants.HardwarePorts.intakeFollowerMotor, MotorType.kBrushless);
        m_followerMotor.follow(m_leaderMotor, true);
    }

    // these methods are all pretty straighforward to understand
    // they do what their name suggests

    public void setState(IntakeStates state) {
        this.intakeState = state;
        m_solenoid.set(intakeState.deployed);
        final double offset = 0.75;
        setSpeed(offset * intakeState.direction);

        if(state.direction==1) {s_Lights.setSelected(6);}
        if(state.direction==-1) {s_Lights.grabbed = false; s_Lights.setNormal();} // i think this should work
    }

    public void setSpeed(double speed) {
        m_leaderMotor.set(speed);
    }

    public boolean getIntakeDeployed() {
        return intakeState.deployed;
    }

    public String getIntakePiece() {
        return intakeState.piece;
    }

    private double coneThreshold = 7.8;

    // current limiting methods to detect when a game piece has been intaked, auto stops the intake
    public boolean hasCone() {
        return m_leaderMotor.getOutputCurrent() > coneThreshold || m_followerMotor.getOutputCurrent() > coneThreshold;
    }

    public double cubeThreshold = 7.8; 

    public boolean hasCube() {
        return m_leaderMotor.getOutputCurrent() > coneThreshold || m_followerMotor.getOutputCurrent() > coneThreshold;
    }

    public boolean motorStopped() {
        return intakeState == IntakeStates.OFF_CLOSED_CONE || intakeState == IntakeStates.OFF_CLOSED_CUBE;
    }

    public boolean stopMovingOnAuto(){
        return intakeState == IntakeStates.OFF_CLOSED_CONE || intakeState == IntakeStates.OFF_CLOSED_CUBE || s_Swerve.getPose().getX() > 7;
    }

    // private double layedConeThreshold = 0;

    // public boolean hasLayedCone() {
    // double currentVolt = m_leaderMotor.getOutputCurrent();
    // return currentVolt > layedConeThreshold;
    // }

    boolean tempHasCube = false;

    @Override
    public void periodic() {
        SmartDashboard.putString("intake piece", getIntakePiece());
        SmartDashboard.putNumber("intake current", m_leaderMotor.getOutputCurrent());
        // SmartDashboard.putBoolean("intake deployed", getIntakeDeployed());

        // if the current for a piece has been detected for 10 loops in a row then auto stop the intake
        // this make it so that it does not stop when the motor spikes when it first starts moving because
        // that spike does not last that long
        if (hasCone()) {
            coneCounter++;
        } else {
            coneCounter = 0;
        }

        if (hasCube()) {
            cubeCounter++;
        } else {
            cubeCounter = 0;
        }

        if (intakeState == IntakeStates.ON_OPEN_CUBE) {
            if (cubeCounter > 10) {
                CommandScheduler.getInstance()
                        .schedule(new WaitCommand(0.0).andThen(new SetIntake(IntakeStates.OFF_CLOSED_CUBE)));
            }
        }

        if (intakeState == IntakeStates.ON_CLOSED_CONE) {
            if (coneCounter > 10) {
                CommandScheduler.getInstance()
                        .schedule(new WaitCommand(0.0).andThen(new SetIntake(IntakeStates.OFF_CLOSED_CONE)));
            }
        }
    }
}
