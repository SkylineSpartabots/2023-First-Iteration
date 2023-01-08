/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;


import com.ctre.phoenix.motorcontrol.StickyFaults;

import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class TalonSRXUtil {

    public static void setCurrentLimit(final LazyTalonSRX talon, int amps) {
        PheonixUtil.checkError(talon.configPeakCurrentLimit(amps, Constants.kTimeOutMs), 
                "Failed to set peak current limit for " + talon.getName(), true);
        PheonixUtil.checkError(talon.configContinuousCurrentLimit(amps, Constants.kTimeOutMs),
                 "Failed to set continious current limit for " + talon.getName(), true);
        PheonixUtil.checkError(talon.configPeakCurrentDuration(0, Constants.kTimeOutMs), 
                "Failed to set peak current duration for " + talon.getName(), true);
        talon.enableCurrentLimit(true);
    }

    public static void testFaults(final LazyTalonSRX talon) {
        StickyFaults faults = new StickyFaults();
        PheonixUtil.checkError(talon.getStickyFaults(faults), "FAILED TO RETRIEVE FAULTS FROM " + talon.getName(), true);
        boolean hasFaults = false;

        if(faults.UnderVoltage) {
            TelemetryUtil.print(talon.getName() + " IS UNDER POWERED", PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(faults.ResetDuringEn) {
            TelemetryUtil.print(talon.getName() + " RESET WHILE ENABLED", PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(hasFaults) {
            PheonixUtil.checkError(talon.clearStickyFaults(), "FAILED TO RESET FAULTS FOR " + talon.getName(), true);
        }
        
    }

    public static void testSlaveFaults(LazyTalonSRX slaveTalon, LazyTalonSRX masterTalon) {
        StickyFaults faults = new StickyFaults();
        PheonixUtil.checkError(slaveTalon.getStickyFaults(faults), "FAILED TO RETRIEVE FAULTS FROM " + 
            slaveTalon.getName(), true);
        boolean hasFaults = false;

        if(faults.UnderVoltage) {
            TelemetryUtil.print(slaveTalon.getName() + " IS UNDER POWERED", PrintStyle.ERROR, true);
            hasFaults = true;
        }

        if(faults.ResetDuringEn) {
            TelemetryUtil.print(slaveTalon.getName() + " RESET WHILE ENABLED", PrintStyle.ERROR, true);
            slaveTalon.follow(masterTalon);
            hasFaults = true;
        }

        if(hasFaults) {
            PheonixUtil.checkError(slaveTalon.clearStickyFaults(), "FAILED TO CLEAR FAULTS FOR " 
                + slaveTalon.getName(), true);
        }
    }

 
}
