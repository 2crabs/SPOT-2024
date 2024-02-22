// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.math;

import java.util.List;

/** Add your docs here. */
public class MathUtils {
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    public static double distance(double x1, double x2, double y1, double y2, double z1, double z2) { //3d distance calc
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2) + Math.pow(z1 - z2, 2));
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

    public static double bringAngleToUnitCircle(double angleDeg) {
        while(angleDeg < 0) {
            while(angleDeg<0) {
                angleDeg += 360;
            }
        }
        while(angleDeg >= 360) {
            while(angleDeg >= 360) {
                angleDeg -= 360;
            }
        }
        return angleDeg;
    }

    /**
     * Rotates a point
     * 
     * @return returns and array with two values, [0] being the x and [1] being the y
     */
    public static double[] rotatePoint(double x, double y, double angle) {
        double[] rotatedPoint = new double[2];
        rotatedPoint[0] = x * Math.cos(Math.toRadians(angle)) - y * Math.sin(Math.toRadians(angle));
        rotatedPoint[1] = x * Math.sin(Math.toRadians(angle)) + y * Math.cos(Math.toRadians(angle));
        return rotatedPoint;
    }

    public static double angleBetweenLines(double x1, double y1, double z1, double x2, double y2, double z2) {
        double dotProduct = x1 * x2 + y1 * y2 + z1 * z2;
        return Math.toDegrees(Math.acos(dotProduct / (Math.abs(distance(0, x1, 0, y1, 0, z1) * distance(0, x2, 0, y2, 0, z2)))));
    }

    public static double angleOfLine(double x1, double y1, double x2, double y2) {
        return Math.atan((y2-y1)/(x2-x1));
    }

    public static double lawOfSinesForAngle(double angle, double a, double b) {
        return Math.toDegrees(Math.asin((b * Math.sin(Math.toRadians(angle))) / a));
    }

    public static double squareKeepSign(double d) {
        return d * d * Math.signum(d);
    }

    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double valueIn : list) {
            result &= epsilonEquals(valueIn, value, epsilon);
        }
        return result;
    }
}
