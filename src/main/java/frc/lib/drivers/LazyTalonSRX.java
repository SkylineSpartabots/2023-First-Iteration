/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class LazyTalonSRX extends TalonSRX {
    
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;
    private String mName;

    protected LazyTalonSRX mMaster = null;

    public LazyTalonSRX(String name, int deviceID) {
        super(deviceID);
        mName = name;
    }

    public double getLastSet() {
        return mLastSet;
    }

    public String getName() {
        return mName;
    }

    public void setMaster(final LazyTalonSRX master) {
        mMaster = master;
        super.set(ControlMode.Follower, master.getDeviceID());
    }

    public LazyTalonSRX getMaster() {
        return mMaster;
    }

    @Override
    public void set(ControlMode mode, double outputValue) {
        if(mode != mLastControlMode || outputValue != mLastSet) {
            mLastControlMode = mode;
            mLastSet = outputValue;
            super.set(mLastControlMode, mLastSet);
        }
    }

    @Override
    public String toString() {
        return getName() + "-> Output Power: " + mLastSet;
    }

}
