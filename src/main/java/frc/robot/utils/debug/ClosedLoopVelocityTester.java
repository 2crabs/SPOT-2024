// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.debug;

import frc.robot.Constants;

/** Add your docs here. */
public class ClosedLoopVelocityTester {
    private final double dv;
    private double currentSetPoint = 0;
    private double targetSetPoint = 0;

    public ClosedLoopVelocityTester(double acceleration) {
        dv = acceleration * 0.002; // Change per frame
    }

    public void setSetPointTarget(double setPoint) {
        targetSetPoint = setPoint;
    }

    public void setSetPointTarget(double setPoint, double currentSpeed) {
        targetSetPoint = setPoint;
        currentSetPoint = currentSpeed;
    }

    public void reset() {
        currentSetPoint = 0;
        targetSetPoint = 0;
    }

    /** Call every frame */
    public double getSetPoint() {
        if (targetSetPoint > currentSetPoint) {
            currentSetPoint += dv;
        }
        if (currentSetPoint > targetSetPoint) {
            currentSetPoint = targetSetPoint;
        }
        else if (targetSetPoint < currentSetPoint) 
        {
            currentSetPoint -= dv;
        }
        if (currentSetPoint < targetSetPoint) {
            currentSetPoint = targetSetPoint;
        }
        return currentSetPoint;
    }

    public double getSetPointTarget() {
        return targetSetPoint;
    }
}
