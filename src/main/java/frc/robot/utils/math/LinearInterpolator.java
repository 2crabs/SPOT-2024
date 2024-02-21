// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.math;

import java.util.Arrays;

/** Linear Interpolator Based On Table */
public class LinearInterpolator {

    private double[][] table;
    private boolean initialized = false;

    /**
     * create linearInterpolator class
     * 
     * "data" is a table of mappings to be interpolated
     */
    public LinearInterpolator(double[][] data) {
        buildTable(data);
    }

    public boolean isInitialized() {
        return initialized;
    }

    private void buildTable(double[][] data) {
        int rows = data.length;
        if (rows < 1) {
            System.out.println("LinearInterpolator: needs at least one data point");
            return;
        }
        int cols = data[0].length;
        if (cols != 2) {
            System.out.println("LinearInterpolator: needs only 2 columns");
            return;
        }

        table = new double[rows][cols];
        for (int x = 0; x < data.length; x++) {
            for (int y = 0; y < data[x].length; y++) {
                table[x][y] = data[x][y];
            }
        }
        Arrays.sort(table, (a, b) -> Double.compare(a[0], b[0]));
        initialized = true;
    }

    /**
     * return interpolated value of x
     * 
     * If the value of x is in the table, that value is returned.
     * 
     * If the value of x is not in the table, The closest two values are used and y
     * is interpolated.
     */
    public double getInterpolatedValue(double x) {

        if (!initialized) {
            System.out.println("ERROR: linearInterpolator number of columns should be 2");
            return 0.0;
        }

        int index = 0;
        for (index = 0; index < table.length; index++) {
            if (table[index][0] >= x) {
                break;
            }
        }

        if (index >= table.length) {
            return table[table.length - 1][1];
        }

        double highY = table[index][1];
        double highX = table[index][0];
        if ((highX == x) || (index == 0)) {
            return highY;
        }
        double lowY = table[index - 1][1];
        double lowX = table[index - 1][0];

        return (lowY + (x - lowX) * (highY - lowY) / (highX - lowX));
    }
}
