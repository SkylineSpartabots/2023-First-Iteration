/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.lib.drivers;

import com.ctre.phoenix.ErrorCode;

import frc.lib.util.TelemetryUtil;
import frc.lib.util.TelemetryUtil.PrintStyle;

public class PheonixUtil {
    public static void checkError(ErrorCode errorCode, String message, boolean log) {
        if (errorCode != ErrorCode.OK) {
            TelemetryUtil.print(message + " " + errorCode, PrintStyle.ERROR, log);
        }
    }
}
