package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.AutomaticScoringSelector;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Swerve extends SubsystemBase {
    private static Swerve instance;
    
    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private SwerveDriveOdometry swerveOdometry;
    private Pigeon2 gyro;
    private SwerveModule[] mSwerveMods;
    public Supplier<Pose2d> poseSupplier = () -> getPose();
    public Consumer<Pose2d> poseConsumer = a -> {resetOdometry(a);};
    public Consumer<ChassisSpeeds> chassisConsumer = a -> {
        autoDrive(a, true);
    };
    public BooleanSupplier isPathRunningSupplier = () -> pathInProgress();
    
    public Swerve() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID, "2976 CANivore");
        gyro.configFactoryDefault();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
                new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
                new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
                new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void autoDrive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    // /* Used by SwerveControllerCommand in Auto */
    // public void setModuleStates(SwerveModuleState[] desiredStates) {
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    //     for (SwerveModule mod : mSwerveMods) {
    //         mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    //     }
    // }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
        gyro.setYaw(pose.getRotation().getDegrees());
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(normalize(gyro.getYaw()));
    }

    public double getPitch() {
        return gyro.getRoll();
    }

    public BooleanSupplier inScoringPosition(Pose2d pose){
        return new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return getPose() == pose;
            }
        };
    }

    public static double normalize(double deg) {
        double angle = deg % 360;
        if (angle < -180) {
            angle = 180 - (Math.abs(angle) - 180);
        } else if (angle > 180) {
            angle = -180 + (Math.abs(angle) - 180);
        }
        return angle;
    }

    public boolean pathInProgress() {
        return !getDefaultCommand().isScheduled();
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());
        // AutomaticScoringSelector.getInstance().updateShuffleboard();
        SmartDashboard.putNumber("gyro-pitch", getPitch());
        SmartDashboard.putNumber("gyro-rot", getYaw().getDegrees());
        SmartDashboard.putNumber("odo-rot", getPose().getRotation().getDegrees());
        SmartDashboard.putNumber("x-pos", getPose().getX());
        SmartDashboard.putNumber("y-pos", getPose().getY());
        SmartDashboard.putBoolean("is OTF running", pathInProgress());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

    }

}