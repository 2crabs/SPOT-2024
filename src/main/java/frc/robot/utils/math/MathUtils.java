// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.math;

/** Add your docs here. */
public class MathUtils {
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double average(double[] data) {
        double mean = 0;
        for (double dataI : data) {
            mean += dataI;
        }
        mean /= data.length;
        return mean;
    }

    public static double stdDev(double[] data) {
        if (data.length == 0 || data.length == 1) return 0;

        double average = average(data);

        double total = 0;
        for (double datum : data) {
            total += Math.pow(datum - average, 2);
        }
        return Math.sqrt(total / (data.length - 1));
    }
}
