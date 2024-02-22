// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.math;

import java.util.ArrayList;

/** Add your docs here. */
public class MovingAverage {
    ArrayList<Double> values = new ArrayList<Double>();
    int maxSize;

    public MovingAverage(int maxSize) {
        this.maxSize = maxSize;
    }

    public void addNumber(double newNumber) {
        values.add(newNumber);
        if (values.size() > maxSize) {
            values.remove(0);
        }
    }

    public double getAverage() {
        double total = 0;

        for (double number : values) {
            total += number;
        }

        return total / values.size();
    }

    public int getSize() {
        return values.size();
    }

    public boolean isUnderMaxSize() {
        return getSize() < maxSize;
    }

    public void clear() {
        values.clear();
    }
}
