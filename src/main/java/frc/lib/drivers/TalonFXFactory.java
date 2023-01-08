/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and BACK_RIGHTared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Constants;

public class TalonFXFactory {

    public final static int kTimeoutMs = Constants.kTimeOutMs;

    public static class Configuration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public double NEUTRAL_DEADBAND = 0.04;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;

        public InvertType INVERTED = InvertType.None;

        public TalonFXFeedbackDevice DEVICE = TalonFXFeedbackDevice.IntegratedSensor;

        public SupplyCurrentLimitConfiguration INPUT_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20,
                0);
        public StatorCurrentLimitConfiguration OUTPUT_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 20, 20,
                0);

        public MotorCommutation COMMUTATION = MotorCommutation.Trapezoidal;

        public SensorInitializationStrategy INITIALIZATION_STRATEGY = SensorInitializationStrategy.BootToZero;
        public boolean SENSOR_PHASE = false;
        public double SENSOR_FEEDBACK_COEFFECIENT = 256 / Math.PI;
        //for gear reduction x (input) -> y (output), coeffecient = 256 * x / Math.PI * y;

        public int CONTROL_FRAME_PERIOD_MS = 5;
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
        public int GENERAL_STATUS_FRAME_RATE_MS = 5;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;

        public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_WINDOW = 64;

    }

    private static Configuration kDefaultConfiguration = new Configuration();
    private static Configuration kSlaveConfiguration = new Configuration();

    static {
        kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 25;
        kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 35;
        kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 5000;
        kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        kSlaveConfiguration.DEVICE = TalonFXFeedbackDevice.None;
    }

    public static LazyTalonFX createDefaultFalcon(String name, int deviceID) {
        return createFalcon(name, deviceID, kDefaultConfiguration);
    }

    public static LazyTalonFX createSlaveFalcon(String name, int deviceID, int masterID) {
        final LazyTalonFX falcon = createFalcon(name, deviceID, kSlaveConfiguration);
        falcon.set(TalonFXControlMode.Follower, masterID);
        return falcon;
    }

    public static LazyTalonFX createFalcon(String name, int deviceID, Configuration config) {
        LazyTalonFX falcon = new LazyTalonFX(name, deviceID);
        falcon.set(ControlMode.PercentOutput, 0.0);

        falcon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        falcon.clearMotionProfileHasUnderrun();
        falcon.clearMotionProfileTrajectories();

        PheonixUtil.checkError(falcon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs), name +
             " failed to configure neutral deadband on init", false);
        
        falcon.configNominalOutputForward(0.0);
        falcon.configNominalOutputReverse(0.0);
        falcon.configPeakOutputForward(1.0);
        falcon.configPeakOutputReverse(-1.0);

        falcon.configVoltageCompSaturation(0.0);
        falcon.configVoltageMeasurementFilter(32);
        falcon.enableVoltageCompensation(false);

        falcon.selectProfileSlot(0, 0);

        PheonixUtil.checkError(falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,
                config.GENERAL_STATUS_FRAME_RATE_MS, kTimeoutMs), name + 
                " failed to set general status frame rate on init", true);
        
        PheonixUtil.checkError(falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,
                config.FEEDBACK_STATUS_FRAME_RATE_MS, kTimeoutMs), name + 
                " failed to set feedback status frame rate on init", true);

        PheonixUtil.checkError(falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature,
                config.QUAD_ENCODER_STATUS_FRAME_RATE_MS, kTimeoutMs), name +
                " failed to set quad encoder frame rate on init", false);
            
        PheonixUtil.checkError(falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat,
                config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS, kTimeoutMs), name + 
                " failed to set faults and temp update rate on init", false);
        
        PheonixUtil.checkError(falcon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth,
                config.PULSE_WIDTH_STATUS_FRAME_RATE_MS, kTimeoutMs), name + 
                " failed to set pulse width status update rate on init", false);

        PheonixUtil.checkError(falcon.setControlFramePeriod(ControlFrame.Control_3_General, config.CONTROL_FRAME_PERIOD_MS),
            name + " failed to set general control frame period on init", true);

        PheonixUtil.checkError(falcon.configStatorCurrentLimit(config.OUTPUT_CURRENT_LIMIT, kTimeoutMs), name +
             " failed to set output current limit on init", true);

        PheonixUtil.checkError(falcon.configSupplyCurrentLimit(config.INPUT_CURRENT_LIMIT, kTimeoutMs), name + 
             " failed to set input current limit on init", true);
        PheonixUtil.checkError(falcon.configVelocityMeasurementPeriod(config.VELOCITY_MEASUREMENT_PERIOD, kTimeoutMs), name +
             " failed to set velocity meas. period on init", true);

        PheonixUtil.checkError(falcon.configVelocityMeasurementWindow(config.VELOCITY_MEASUREMENT_ROLLING_WINDOW, kTimeoutMs), name +
             " failed to set velocity measurement window on init", true);    
        
        PheonixUtil.checkError(falcon.clearStickyFaults(), name + 
             " failed to clear sticky faults on init", true);

        /*PheonixUtil.checkError(falcon.configSelectedFeedbackSensor(config.DEVICE, 0, kTimeoutMs),
            name + " failed to set feedback sensor on init", true);*/

        PheonixUtil.checkError(falcon.configSelectedFeedbackCoefficient(config.SENSOR_FEEDBACK_COEFFECIENT, 0, kTimeoutMs),
            name + " failed to set sensor coeffecient on init", true);
        
        PheonixUtil.checkError(falcon.configOpenloopRamp(config.OPEN_LOOP_RAMP_RATE, kTimeoutMs), 
            name + " failed to set open loop ramp rate on init", true);
        
        PheonixUtil.checkError(falcon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs), 
            name + " failed to set closed loop ramp rate on init", true);
        
        PheonixUtil.checkError(falcon.configClosedloopRamp(config.CLOSED_LOOP_RAMP_RATE, kTimeoutMs), 
            name + " failed to set open loop ramp rate on init", true);
        
        PheonixUtil.checkError(falcon.configForwardSoftLimitEnable(false, kTimeoutMs), 
            name + " failed to disable fwd soft limit", true);
        
        PheonixUtil.checkError(falcon.configReverseSoftLimitEnable(false, kTimeoutMs), 
            name + " failed to disable reverse soft limit", true);
        
        falcon.setInverted(config.INVERTED);
        falcon.setSensorPhase(config.SENSOR_PHASE);
    
        return falcon;
    }

}