/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class LazyTalonFX extends TalonFX {

    protected double mLastSet = Double.NaN;
    protected TalonFXControlMode mLastControlMode = null;
    private String mName;

    protected LazyTalonFX mMaster = null;

    public LazyTalonFX(String name, int deviceID) {
        super(deviceID);
        mName = name;
    }

    public double getLastSet() {
        return mLastSet;
    }

    public void setName(String name) {
        mName = name;
    }

    public String getName() {
        return mName;
    }

    public LazyTalonFX getMaster() {
        return mMaster;
    }

    public void setMaster(final LazyTalonFX leader) {
        mMaster = leader;
        super.set(ControlMode.Follower, leader.getDeviceID());
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        if(mode != mLastControlMode || value != mLastSet) {
            mLastControlMode = mode;
            mLastSet = value;
            super.set(mode, value);
        }
    }

    @Override
    public String toString() {
        return getName() + " -> Output Power: " + mLastSet; 
    }

}