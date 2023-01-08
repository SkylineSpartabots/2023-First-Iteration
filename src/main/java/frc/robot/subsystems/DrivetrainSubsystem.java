package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.TeleopDriveCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.Ports;

import java.util.function.Consumer;
import java.util.function.Supplier;


public class DrivetrainSubsystem extends SubsystemBase {
    private static DrivetrainSubsystem m_instance = null;
    private static double expectedVelocity;

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    private final SwerveDriveOdometry m_odometry;
    private final SimpleMotorFeedforward m_feedforward;
    
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    Consumer<ChassisSpeeds> consume = a -> {
        drive(a);
        applyDrive();
    };

    Supplier<Pose2d> supply = () -> getPose();
    // These are our modules. We initialize them in the constructor.
    private final SwerveModule m_frontLeftModule;
    private final SwerveModule m_frontRightModule;
    private final SwerveModule m_backLeftModule;
    private final SwerveModule m_backRightModule;
    
    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private Field2d m_field = new Field2d();
    private Field2d m_hub = new Field2d();
    
    public static DrivetrainSubsystem getInstance() {
      if (m_instance == null) {
          m_instance = new DrivetrainSubsystem();
      }
      return m_instance;
  }

    public DrivetrainSubsystem() {
        setDefaultCommand(new TeleopDriveCommand(this));

        SmartDashboard.putData(m_field);
        //SmartDashboard.putData(m_hub);
        //ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        Mk4ModuleConfiguration driveConfiguration = new Mk4ModuleConfiguration();
        driveConfiguration.setDriveCurrentLimit(40);
        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
        //        tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                driveConfiguration,
                Mk4SwerveModuleHelper.GearRatio.L2,
                // Port ID of drive motor, steer motor, steer encoder offset
                Ports.FRONT_LEFT_DRIVE, Ports.FRONT_LEFT_STEER,
                Ports.FRONT_LEFT_STEER_ENCODER, Ports.FRONT_LEFT_OFFSET);
        // Steer Encoder Offset: This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)


        // We will do the same for the other modules
        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                //tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
                driveConfiguration,
                Mk4SwerveModuleHelper.GearRatio.L2, Ports.FRONT_RIGHT_DRIVE, Ports.FRONT_RIGHT_STEER,
                Ports.FRONT_RIGHT_STEER_ENCODER, Ports.FRONT_RIGHT_OFFSET);

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                //tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
                driveConfiguration,
                Mk4SwerveModuleHelper.GearRatio.L2, Ports.BACK_LEFT_DRIVE, Ports.BACK_LEFT_STEER,
                Ports.BACK_LEFT_STEER_ENCODER, Ports.BACK_LEFT_OFFSET);

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                //tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
                driveConfiguration,
                Mk4SwerveModuleHelper.GearRatio.L2, Ports.BACK_RIGHT_DRIVE, Ports.BACK_RIGHT_STEER,
                Ports.BACK_RIGHT_STEER_ENCODER, Ports.BACK_RIGHT_OFFSET);

        m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyroscopeRotation());
        m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
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

    public Field2d getField() {
        return m_field;
    }
    public void setHubPosition(double hubX, double hubY){
        m_hub.setRobotPose(hubX, hubY, new Rotation2d(0));
    }

    //sets Gyroscope to 0
    public void zeroGyroscope() {
        m_navx.zeroYaw();
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(normalize(-m_navx.getAngle()));
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    //resets to 0,0
    public void resetOdometry() {
        m_navx.reset();
        m_navx.setAngleAdjustment(0);
        m_odometry.resetPosition(new Pose2d(), new Rotation2d(0));
    }

    //resets to start position (for blue four/five ball auto)
    public void resetFromStart() {
        final double startX = 7.82;
        final double startY = 2.97;
        final double startRot = -111;
        resetOdometryFromPosition(startX, startY, startRot);
    }

    //resets from offset
    public void resetOdometryFromPosition(double x, double y, double rot) {
        m_navx.reset();
        m_navx.setAngleAdjustment(-rot);
        m_odometry.resetPosition(new Pose2d(x, y, new Rotation2d(rot)), new Rotation2d(rot));
    }

    //resets from offset
    public void resetOdometryFromPosition(double x, double y) {
        m_odometry.resetPosition(new Pose2d(x, y, getGyroscopeRotation()), getGyroscopeRotation());
    }

    public void resetOdometryFromPosition(Pose2d pose) {
        m_navx.reset();
        m_navx.setAngleAdjustment(-pose.getRotation().getDegrees());
        m_odometry.resetPosition(pose, pose.getRotation());
    }

    public void resetOdometryFromReference() {
        Translation2d current = getPose().getTranslation();
        double minError = 100d;
        Translation2d newPos = null;
        for (Translation2d ref : Constants.kReferenceTranslations) {
            double errorX = Math.abs(ref.getX() - current.getX());
            double errorY = Math.abs(ref.getY() - current.getY());
            double error = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2));
            if (error < minError) {
                newPos = ref;
                minError = error;
            }
        }
        resetOdometryFromPosition(new Pose2d(newPos, getGyroscopeRotation()));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
    }

    public boolean getNavxConnected() {
        return m_navx.isConnected();
    }

    public double getRealVelocity() {
        //FIND BETTER WAY TO DO THIS
        //averaging module speeds
        return (m_frontLeftModule.getDriveVelocity() + m_frontRightModule.getDriveVelocity() +
                m_backLeftModule.getDriveVelocity() + m_backRightModule.getDriveVelocity()) / 4;
    }

    public double getExpectedVelocity() {
        return expectedVelocity;
    }

    @Override
    public void periodic() {
        applyDrive();
        SmartDashboard.putNumber("Rotation", getGyroscopeRotation().getDegrees());
        SmartDashboard.putBoolean("IsCalibrating", m_navx.isCalibrating());

        SmartDashboard.putNumber("FrontLeft", new CANCoder(9).getAbsolutePosition());
        SmartDashboard.putNumber("BackLeft", new CANCoder(10).getAbsolutePosition());
        SmartDashboard.putNumber("BackRight", new CANCoder(11).getAbsolutePosition());
        SmartDashboard.putNumber("FrontRight", new CANCoder(12).getAbsolutePosition());


        /*SmartDashboard.putNumber("FLSteer", RobotContainer.getPDP().getCurrent(4));
        SmartDashboard.putNumber("FLDrive", RobotContainer.getPDP().getCurrent(6));
        SmartDashboard.putNumber("BLSteer", RobotContainer.getPDP().getCurrent(5));
        SmartDashboard.putNumber("BLDrive", RobotContainer.getPDP().getCurrent(7));
        SmartDashboard.putNumber("FRSteer", RobotContainer.getPDP().getCurrent(15));
        SmartDashboard.putNumber("FRDrive", RobotContainer.getPDP().getCurrent(13));
        SmartDashboard.putNumber("BRDrive", RobotContainer.getPDP().getCurrent(12));
        SmartDashboard.putNumber("BRSteer", RobotContainer.getPDP().getCurrent(14));*/

    }

    public void applyDrive() {
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxSpeedMetersPerSecond);

        m_frontLeftModule.set(getVoltageByVelocity(states[0].speedMetersPerSecond), states[0].angle.getRadians());
        m_frontRightModule.set(getVoltageByVelocity(states[1].speedMetersPerSecond), states[1].angle.getRadians());
        m_backLeftModule.set(getVoltageByVelocity(states[2].speedMetersPerSecond), states[2].angle.getRadians());
        m_backRightModule.set(getVoltageByVelocity(states[3].speedMetersPerSecond), states[3].angle.getRadians());
        // m_frontLeftModule.set(states[0].speedMetersPerSecond / m_driveConstants.kMaxSpeedMetersPerSecond * MAX_VOLTAGE , states[0].angle.getRadians());
        // m_frontRightModule.set(states[1].speedMetersPerSecond / m_driveConstants.kMaxSpeedMetersPerSecond * MAX_VOLTAGE, states[1].angle.getRadians());
        // m_backLeftModule.set(states[2].speedMetersPerSecond / m_driveConstants.kMaxSpeedMetersPerSecond * MAX_VOLTAGE, states[2].angle.getRadians());
        // m_backRightModule.set(states[3].speedMetersPerSecond / m_driveConstants.kMaxSpeedMetersPerSecond * MAX_VOLTAGE, states[3].angle.getRadians());

        //if (DriverStation.isAutonomous()) {
            m_odometry.update(getGyroscopeRotation(),
                    new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
                    new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle())),
                    new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle())),
                    new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle())));
            
            Pose2d pose = m_odometry.getPoseMeters();

            SmartDashboard.putNumber("X Position", pose.getTranslation().getX());
            SmartDashboard.putNumber("Y Position", pose.getTranslation().getY());
            
            m_field.setRobotPose(pose);
        //}
    }

    public double getVoltageByVelocity(double targetVelocity) {
        return m_feedforward.calculate(targetVelocity * DriveConstants.kVelocityGain);
    }

    
  public static double distanceFromHub(double targetX, double targetY){
    return calculateDistance(
      DrivetrainSubsystem.getInstance().getPose().getX(), DrivetrainSubsystem.getInstance().getPose().getY(), targetX,targetY);
  }
  public static double calculateDistance(double x1, double y1, double x2, double y2){
    return Math.sqrt(Math.pow(x1-x2,2) + Math.pow(y1-y2,2));
  }

  public static double findAngle(Pose2d currentPose, double toX, double toY, double offsetDeg){
      double deltaY = (toY - currentPose.getY());
      double deltaX = (toX - currentPose.getX());

      double absolute = Math.toDegrees(Math.atan2(deltaY, deltaX));
      return normalize(absolute + offsetDeg);
  
    }

 public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    PIDController xController = new PIDController(0.0, 0, 0.0);
    PIDController yController = new PIDController(0.0, 0, 0.0);
    PIDController thetaController = new PIDController(0.0, 0, 0.0);
    return new SequentialCommandGroup(
            new InstantCommand(() -> {
            // Reset odometry for the first path you run during auto
            if(isFirstPath){
                this.resetOdometryFromPosition(traj.getInitialHolonomicPose());
            }
            }),
            new PPSwerveControllerCommand(
                traj, 
                this.supply,
                xController,
                yController,
                thetaController,
                this.consume
            )
        );
    }   
}

