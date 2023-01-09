// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.util.Color;
import com.kauailabs.navx.frc.AHRS;


//holds robot-wide constants
public final class Constants {
    public static final AHRS gyro = new AHRS(SPI.Port.kMXP); // FIXME change to correct


    // timeout for CAN commands and error checking
    public static final int kTimeOutMs = 10;
    public static final double kMinimumBatteryVoltage = 12;
    public static final int kCanDeviceCount = 0;
    
    public static final double shooterFixed = 10500;
    public static final double shooterRamped = 11000;
    public static final double shooterIdle = 10500;
    public static final double shooterEjection = 8000;

    public static final double indexerUp = 0.70;
    public static final double indexerDown = -0.5;

    public static final double intakeOn = 0.75;
    public static final double intakeReverse = -0.5;

    public static final double waitBetweenShots = 0.25;

    public static final double climbUp = 0.99;
    public static final double climbDown = -0.99; //usually 0.8, but potential issues having joystick moved to a full position
    public static final double pivotUp = -0.4;
    public static final double pivotDown = 0.4;
    

    
    public static final Translation2d targetHudPosition = new Translation2d(8.23, 4.165);

    public static final Color kColorSensorBlueIntake = new Color(0.19,0.43, 0.37);
    public static final Color kColorSensorRedIntake = new Color(0.44, 0.38, 0.16);

    public static final Color allianceColorIntake = kColorSensorBlueIntake;
    
    public static final double kColorSensorLoadingDistance = 70;

    //Dummy values, need to find/calculate
    public static final List<Translation2d> kReferenceTranslations = List.of(
                new Translation2d(1, 0),
                new Translation2d(3, 3)
        );

    public static final class DriveConstants {
        public static final double kTrackWidth = 0.4953;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.4953;
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;
        //Calculated via SysId
        public static final double ksVolts = 0.74397; //before 3/1: 70541, 33259, 016433
        public static final double kvVoltSecondsPerMeter = 0.33778;
        public static final double kaVoltSecondsSquaredPerMeter = 0.016934;
        //Tuned to taste for desired max velocity
        public static final double kVelocityGain = 6;
        // The maximum voltage that will be delivered to the drive motors.
        // This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
        public static final double kMaxVoltage = 12.0;
        // The maximum velocity of the robot in meters per second.
        // This is a measure of how fast the robot should be able to drive in a straight line.
        public static final double kMaxSpeedMetersPerSecond =  
        6380.0 / 60.0 *
               SdsModuleConfigurations.MK4_L2.getDriveReduction() *
               SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
       // need measure on robot
       public static final double kMaxAccelerationMetersPerSecondSquared = 10; 
       //The maximum angular velocity of the robot in radians per second.
       //This is a measure of how fast the robot can rotate in place.
       // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
       public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
              Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = kMaxAngularSpeedRadiansPerSecond;

        public static final double kpRotation = 0.1;
        public static final double kiRotation = 0.0;
        public static final double kdRotation = 0;
        public static final TrapezoidProfile.Constraints kRotationConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class Ports{
        public static final int FRONT_LEFT_DRIVE = 2;
        public static final int FRONT_LEFT_STEER = 1;
        public static final int FRONT_LEFT_STEER_ENCODER = 9;
        public static final double FRONT_LEFT_OFFSET = -Math.toRadians(301.376953125);

        public static final int FRONT_RIGHT_DRIVE = 8;
        public static final int FRONT_RIGHT_STEER = 7;
        public static final int FRONT_RIGHT_STEER_ENCODER = 12;
        public static final double FRONT_RIGHT_OFFSET = -Math.toRadians(65.390625);

        public static final int BACK_LEFT_DRIVE = 4;
        public static final int BACK_LEFT_STEER = 3;
        public static final int BACK_LEFT_STEER_ENCODER = 10;
        public static final double BACK_LEFT_OFFSET = -Math.toRadians(324.66796875);

        public static final int BACK_RIGHT_DRIVE = 6;
        public static final int BACK_RIGHT_STEER = 5;
        public static final int BACK_RIGHT_STEER_ENCODER = 11;
        public static final double BACK_RIGHT_OFFSET = -Math.toRadians(357.890625);

        public static final int MASTER_SHOOTER_MOTOR = 21;
        public static final int FOLLOW_SHOOTER_MOTOR = 22;
        public static final int INTAKE_MOTOR = 31;
        public static final int INDEXER_MOTOR = 32;
        public static final int PIVOT_MOTOR = 23;
        
        public static final int LEFT_CLIMB = 41;
        public static final int RIGHT_CLIMB = 42;
        public static final int LEFT_PIVOT = 44;
        public static final int RIGHT_PIVOT = 43;

    }
}
