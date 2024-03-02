// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import org.opencv.core.Point;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.*;

/** Add your docs here. */
public class Shape {
    private ShapeEnum name;
    private Point center;
    private int points;
    private MatOfPoint contour;

    public Shape(ShapeEnum shapeName, Point shapeCenter, int shapePoints, MatOfPoint shapeContour) {
        center = shapeCenter;
        name = shapeName;
        points = shapePoints;
        contour = shapeContour;
    }

    public ShapeEnum getShapeName() {
        return name;
    }

    public Point getCenter() {
        return center;
    }

    public int getNumberOfPoints() {
        return points;
    }

    public MatOfPoint getContour() {
        return contour;
    }
}