// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  private final NetworkTable m_limelightTable;

  private double targetOffsetX, targetOffsetY, targetArea, targetSkew;
  private double[] botPose;
  private NetworkTableEntry 
    target_valid,
    target_x,
    target_y,
    target_area,
    target_skew,
    pipeline_latency,
    image_latency,
    short_side_length,
    long_side_length,
    x_side_length,
    y_side_length,
    pipeline_get,
    pipeline_set,
    led_mode,
    cam_mode,
    streaming_mode;

  /** Creates a new Vision. */
  public Vision() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    configureEntries();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetOffsetX = m_limelightTable.getEntry("tx").getDouble(0);
    targetOffsetY = m_limelightTable.getEntry("ty").getDouble(0);
    targetArea = m_limelightTable.getEntry("ta").getDouble(0);
    targetSkew = m_limelightTable.getEntry("ts").getDouble(0);

    botPose = m_limelightTable.getEntry("botpose").getDoubleArray(new double[6]);

    SmartDashboard.putNumber("targetXOffset", targetOffsetX);
    SmartDashboard.putNumber("targetYOffset", targetOffsetY);
    SmartDashboard.putNumber("targetAOffset", targetArea);
    SmartDashboard.putNumber("targetSOffset", targetSkew);

  }

  public void setLEDMode(LimelightLEDMode mode) {
    led_mode.setNumber(mode.modeID);
  }

  public LimelightLEDMode getLEDMode() {
    return LimelightLEDMode.fromModeID(led_mode.getNumber(1).intValue());
  }

  public int getCurrentPipeline() {
    return pipeline_get.getNumber(0).intValue();
  }

  public void setPipeline(int pipelineNumber) {
    pipeline_set.setNumber(pipelineNumber);
  }

  public boolean hasValidTarget() {
    return target_valid.getDouble(0) != 0.0;
  }

  public double getLatencyMS() {
    return pipeline_latency.getDouble(0) + image_latency.getDouble(0.0);
  }

  public double getTargetOffsetX() {
    return targetOffsetX;
  }
  public double getTargetOffsetY() {
    return targetOffsetY;
  }
  public double getTargetArea() {
    return targetArea;
  }
  public double getTargetSkew() {
    return targetSkew;
  }

  public double getTargetShortSideLength() {
    return short_side_length.getDouble(0);
  }
  public double getTargetLongSideLength() {
    return long_side_length.getDouble(0);
  }
  public double getTargetXSideLength() {
    return x_side_length.getDouble(0);
  }
  public double getTargetYSideLength() {
    return y_side_length.getDouble(0);
  }

  public Pose2d getBotPose() {
    Rotation2d rotation = new Rotation2d(botPose[5]);
    Translation2d translation = new Translation2d(botPose[0],botPose[2]);
    return new Pose2d(translation, rotation);
  }

  public enum LimelightLEDMode {
    PIPELINE(0), OFF(1), BLINK(2), ON(3);

    public final int modeID;
    private static final LimelightLEDMode[] values = values();

    private LimelightLEDMode(int modeID) {
      this.modeID = modeID;
    }
    
    public static LimelightLEDMode fromModeID(int modeID) {
      for (LimelightLEDMode mode : values) {
        if (mode.modeID == modeID) {
          return mode;
        }
      }
      return null;
    }
  }

  public void configureEntries() {
    target_valid = m_limelightTable.getEntry("tv");
    target_x = m_limelightTable.getEntry("tx");
    target_y = m_limelightTable.getEntry("ty");
    target_area = m_limelightTable.getEntry("ta");

    pipeline_latency = m_limelightTable.getEntry("tl");
    image_latency = m_limelightTable.getEntry("cl");

    short_side_length = m_limelightTable.getEntry("tshort");
    long_side_length = m_limelightTable.getEntry("tlong");
    x_side_length = m_limelightTable.getEntry("thor");
    y_side_length = m_limelightTable.getEntry("tvert");

    pipeline_get = m_limelightTable.getEntry("getpipe");
    pipeline_set = m_limelightTable.getEntry("pipeline");

    led_mode = m_limelightTable.getEntry("ledMode");
    cam_mode = m_limelightTable.getEntry("camMode");
    streaming_mode = m_limelightTable.getEntry("stream");
  }
}
