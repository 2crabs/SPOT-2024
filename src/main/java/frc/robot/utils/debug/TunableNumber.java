// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.debug;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class TunableNumber {
    private String key;
    private double defaultValue;

    public TunableNumber(String dashboardKey) {
        this.key = dashboardKey;
    }

    public double getDefault() {
        return defaultValue;
    }

    public void setDefault(double defaultValue) {
        this.defaultValue = defaultValue;
        SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
    }

    public double get() {
        return SmartDashboard.getNumber(key, defaultValue);
    }
}
