// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import frc.robot.Constants.kDisplay;

/** Add your docs here. */
public class ShapeDetection {
    private double areaMinThreshold;
    private Mat srcGray = new Mat();

    private int maxDetection = 0;

    private List<Shape> foundShapes = new ArrayList<>();

    public ShapeDetection(int maxShapes, double areaMin) {
        maxDetection = maxShapes;
        areaMinThreshold = areaMin;
        foundShapes.clear();
    }

    public void detectShapesFromImage(Mat img) {
        foundShapes.clear();

        if(img.empty()) {
            System.out.println("Cannot Read Camera Output");
        }

        Imgproc.cvtColor(img, srcGray, Imgproc.COLOR_BGR2GRAY);
        Imgproc.blur(srcGray, srcGray, new Size(3, 3));

        List<MatOfPoint> foundContours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(srcGray, foundContours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        foundContours.sort(
            new Comparator<MatOfPoint>() {
                public int compare(MatOfPoint c1, MatOfPoint c2) {          
                    return (int) (Imgproc.contourArea(c1)- Imgproc.contourArea(c2));
                }
            }
        );

        for(MatOfPoint cnt : foundContours) {
            if(Imgproc.contourArea(cnt) > areaMinThreshold) {
                MatOfPoint approx = new MatOfPoint(cnt);
                MatOfPoint2f approxf = new MatOfPoint2f(cnt);

                Imgproc.approxPolyDP(new MatOfPoint2f(cnt), approxf, 0.01 * Imgproc.arcLength(new MatOfPoint2f(cnt), true), true);
                approxf.convertTo(approx, CvType.CV_32S);

                Size contourSize = approx.size();
                int contourPoints = (int)contourSize.height;

                ShapeEnum shapeName = ShapeEnum.NOTHING;

                switch(contourPoints) {
                    case 1:
                        shapeName = ShapeEnum.UNKNOWN;
                    case 2:
                        shapeName = ShapeEnum.UNKNOWN;
                    case 3:
                        shapeName = ShapeEnum.TRIANGLE;
                    case 4:
                        shapeName = ShapeEnum.QUADRILATERAL;
                    case 5:
                        shapeName = ShapeEnum.UNKNOWN;
                    case 6:
                        shapeName = ShapeEnum.CUBE;
                    case 7:
                        shapeName = ShapeEnum.CUBE;
                    case 8:
                        shapeName = ShapeEnum.CYLINDER;
                    default:
                        shapeName = ShapeEnum.CIRCLE;
                }

                Moments moments = Imgproc.moments(cnt);

                Point center = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());

                foundShapes.add(new Shape(shapeName, center, contourPoints, cnt));
            }
        }
    }

    public List<Shape> getFoundShapes() {
        return foundShapes;
    }

    public int getNumberOfShapes() {
        return foundShapes.size();
    }

    public Shape getShape(int index) {
        return foundShapes.get(index);
    }

    public Shape getBiggestShape() {
        return foundShapes.get(0);
    }

    public boolean containsNotes() {
        boolean containsNote = false;
        for(int i = 0; i < foundShapes.size(); i++) {
            if(foundShapes.get(i).getShapeName() == ShapeEnum.CIRCLE) {
                containsNote = true;
            }
        }
        return containsNote;
    }

    public List<Integer> getNoteIndexes() {
        List<Integer> noteIndexes = new ArrayList<>();
        for(int i = 0; i < foundShapes.size(); i++) {
            if(foundShapes.get(i).getShapeName() == ShapeEnum.CIRCLE) {
                noteIndexes.add(i);
            }
        }
        return noteIndexes;
    }

    public Shape getBiggestNote() {
        return foundShapes.get(getNoteIndexes().get(0));
    }

    /**
     * Renders the details of all detected shapes on top of a source image
     * @param src The source image to render details on to
     * @param lineColor The color of the bounding box and center
     * @param detailsColor The colors of the details like name and points
     * @param renderBoundingBox Enable rendering the bounding box
     * @param renderName Enable rendering the name of the shape
     * @param renderCenterPoint Enable rendering the center point as crosshair
     * @param renderOuterPoints Enable rendering the outer points of the bounding box
     * @param renderContourPoints Enable rendering the points on the contour
     */
    public Mat renderShapeDetails(Mat src, Scalar lineColor, Scalar detailsColor, boolean renderBoundingBox, boolean renderName, boolean renderCenterPoint, boolean renderOuterPoints, boolean renderContourPoints) {
        Mat rendered = src;

        for(int i = 0; i < foundShapes.size(); i++) {
            Rect boundingBox = Imgproc.boundingRect(getShape(i).getContour());
            if(renderBoundingBox) {
                Imgproc.rectangle(rendered, boundingBox, detailsColor, kDisplay.LINE_THICKNESS);
                if(renderOuterPoints) {
                    Imgproc.circle(rendered, boundingBox.br(), kDisplay.RENDER_POINT_RADIUS, detailsColor);
                    Imgproc.circle(rendered, new Point(boundingBox.br().x - boundingBox.width, boundingBox.br().y), kDisplay.RENDER_POINT_RADIUS, detailsColor);
                    Imgproc.circle(rendered, boundingBox.tl(), kDisplay.RENDER_POINT_RADIUS, detailsColor);
                    Imgproc.circle(rendered, new Point(boundingBox.tl().x + boundingBox.width, boundingBox.tl().y), kDisplay.RENDER_POINT_RADIUS, detailsColor);
                }
            }
            if(renderContourPoints) {
                MatOfPoint contourPoints = getShape(i).getContour();
                List<Point> pointList = contourPoints.toList();
                for(int j = 0; j < getShape(i).getNumberOfPoints(); j++) {
                    Imgproc.circle(rendered, pointList.get(j), kDisplay.RENDER_POINT_RADIUS, detailsColor);
                }
            }
            if(renderName) {
                Imgproc.putText(rendered, getShape(i).getShapeName().toString(), new Point(
                    (boundingBox.tl().x) + kDisplay.NAME_RENDER_OFFSET.x,
                    boundingBox.tl().y + kDisplay.NAME_RENDER_OFFSET.y
                ), 1, kDisplay.NAME_RENDER_SIZE, detailsColor);
            }
            if(renderCenterPoint) {
                Imgproc.circle(rendered, getShape(i).getCenter(), kDisplay.RENDER_POINT_RADIUS * 2, lineColor);
            }
        }

        return rendered;
    }
}