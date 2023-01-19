package frc.robot.subsystems;

import frc.robot.SwerveModule;
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
    private SwerveDriveOdometry swerveOdometry;
    private Pigeon2 gyro;
    private SwerveModule[] mSwerveMods;
    public Supplier<Pose2d> poseSupplier = () -> getPose();
    public Consumer<Pose2d> poseConsumer = a -> {resetOdometry(a);};
    public Consumer<ChassisSpeeds> chassisConsumer = a -> {
        autoDrive(a, true);
    };
    public BooleanSupplier isPathRunningSupplier = () -> pathInProgress();

    private static Swerve instance;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void autoDrive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

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
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
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

    //system.out.println "Helloworld";
    //system.out.println "my name is tiffany and im the best person in the world";
    //system.out.println "tiffany is the best snack lead in the world";
    //system.out.println "like robotics would not be robotics if it weren't for tiffany ong frfr";
    //system.out.println "hashtag not my prez hashtag not my director of software hashtag not my 
    // director of engineering hashtag not my director of media"
    //system.out.println "honestly like honestly like if i were really to say something like i really 
    // mean it like its something really important but it was like liek but thats crayz"
    //system.out.println("tiffany lifts way way way way way way way way WAYYYY more than ved. VED? 
    // MORE LIKE HE STAYS IN BED! VED? WHAT IS MY BLUD WAFFLING ABOUT RN????????????")
    //system.out.println("i bet ved lifts like 5 pounds max on all machines because of his figure");
    //system.out.println("tiffany lifts 700kg with her brain and her fingers");
}