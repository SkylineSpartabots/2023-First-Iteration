package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static Pose2d targetPosition;

    public static final double stickDeadband = 0.1;

    public static final class Limelight {
        public static final String photonCamName = "OV5647";
        public static final Translation3d cameraOffsets = new Translation3d(
            Units.inchesToMeters(-14.5), // x (front-back) offset
            Units.inchesToMeters(0), // y (right-left) offset
            Units.inchesToMeters(-34) // z (up-down)
        );
        public static final Rotation3d cameraAngleOffsets = new Rotation3d(
            Units.degreesToRadians(0), // x (roll)
            Units.degreesToRadians(0), // y (pitch)
            Units.degreesToRadians(0) // z (yaw)
        );
        public static final Pose3d[] gameAprilTags = {
                new Pose3d(15.51, 1.07, 0.46, new Rotation3d(0, 0, Math.PI)),
                new Pose3d(15.51, 2.74, 0.46, new Rotation3d(0, 0, Math.PI)),
                // new Pose3d(15.51, 4.42, 0.46, new Rotation3d(0, 0, Math.PI)),
                new Pose3d(0, 0, 0.46, new Rotation3d(0, 0, 0)),
                new Pose3d(16.18, 6.75, 0.69, new Rotation3d(0, 0, Math.PI)),
                new Pose3d(0.36, 6.75, 0.69, new Rotation3d(0, 0, 0)),
                new Pose3d(1.03, 4.42, 0.46, new Rotation3d(0, 0, 0)),
                new Pose3d(1.03, 2.74, 0.46, new Rotation3d(0, 0, 0)),
                new Pose3d(1.03, 1.07, 0.46, new Rotation3d(0, 0, 0))
        };

        public static final Pose2d[] gameAprilTags2d = {
            new Pose2d(15.51, 1.07, new Rotation2d(0)),
            new Pose2d(15.51, 2.74, new Rotation2d(0)),
            new Pose2d(15.51, 4.42, new Rotation2d(0)),
            new Pose2d(16.18, 6.75, new Rotation2d(0)),
            new Pose2d(0.36, 6.75, new Rotation2d(0)),
            new Pose2d(1.03, 4.42, new Rotation2d(0)),
            new Pose2d(1.03, 2.74, new Rotation2d(0)),
            new Pose2d(1.03, 1.07, new Rotation2d(0))
        };
    }

    public static final class Arm {
        public static final int extensionMotorID = 20;
    }

    public static final class Swerve {

        public static final int pigeonID = 15;

        public static final COTSFalconSwerveConstants chosenModule = COTSFalconSwerveConstants
                .SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 0.5715;
        public static final double wheelBase = 0.5715;
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */ // DID NOT CHANGE
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */ // DID NOT CHANGE
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(300.2);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(64.7);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(324.7);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(358.8);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

}