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
  private final NetworkTable m_raspberryPiTable;

  // Simple target estimations
  private double targetOffsetX, targetOffsetY, targetArea, targetSkew;
  // 3d estimations from limelight
  private double[] 
    botPose, 
    botPoseTargetSpace,  
    botPoseRed, 
    botPoseBlue, 
    camPoseRobotSpace, 
    camPoseTargetSpace, 
    targetPoseBotSpace, 
    targetPoseCamSpace;
  // Simple note estimations
  private double
    noteOffsetX,
    noteOffsetY,
    noteScreenArea,
    noteDistance;
  // 3d estimations for notes
  private double[]
    noteHomographyMatrix,
    note3dPose;
  private NetworkTableEntry 
    target_valid,
    target_x,
    target_y,
    target_area,
    target_skew,
    target_id,
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
  private NetworkTableEntry
    target_note_x,
    target_note_y,
    target_note_area,
    target_note_homography_matrix,
    target_note_estimated_3d_pose,
    target_note_estimated_distance;

  /** Creates a new Vision. */
  public Vision() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    m_raspberryPiTable = NetworkTableInstance.getDefault().getTable("notevision");
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
    botPoseRed = m_limelightTable.getEntry("botpose_wpired").getDoubleArray(new double[6]);
    botPoseBlue = m_limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    botPoseTargetSpace = m_limelightTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    camPoseTargetSpace = m_limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    camPoseRobotSpace = m_limelightTable.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    targetPoseCamSpace = m_limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]);
    targetPoseBotSpace = m_limelightTable.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    noteOffsetX = target_note_x.getDouble(0);
    noteOffsetY = target_note_y.getDouble(0);
    noteScreenArea = target_note_area.getDouble(0);
    noteDistance = target_note_estimated_distance.getDouble(0);
    noteHomographyMatrix = target_note_homography_matrix.getDoubleArray(new double[9]);
    note3dPose = target_note_estimated_3d_pose.getDoubleArray(new double[6]);

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

  public Pose2d getBotPoseRed() {
    Rotation2d rotation = new Rotation2d(botPoseRed[5]);
    Translation2d translation = new Translation2d(botPoseRed[0],botPoseRed[2]);
    return new Pose2d(translation, rotation);
  }

  public Pose2d getBotPoseBlue() {
    Rotation2d rotation = new Rotation2d(botPoseBlue[5]);
    Translation2d translation = new Translation2d(botPoseBlue[0],botPoseBlue[2]);
    return new Pose2d(translation, rotation);
  }

  public Pose2d getPoseCustomSpace(PoseSpace get, PoseSpace space) {
    Rotation2d rotation = null;
    Translation2d translation = null;
    if(get == PoseSpace.ROBOT) {
      if(space == PoseSpace.TARGET) {
        rotation = new Rotation2d(botPoseTargetSpace[5]);
        translation = new Translation2d(botPoseTargetSpace[0], botPoseTargetSpace[2]);
      }
    }
    if(get == PoseSpace.CAMERA) {
      if(space == PoseSpace.TARGET) {
        rotation = new Rotation2d(camPoseTargetSpace[5]);
        translation = new Translation2d(camPoseTargetSpace[0], camPoseTargetSpace[2]);
      }
      else {
        rotation = new Rotation2d(camPoseRobotSpace[5]);
        translation = new Translation2d(camPoseRobotSpace[0], camPoseRobotSpace[2]);
      }
    }
    if(get == PoseSpace.TARGET) {
      if(space == PoseSpace.ROBOT) {
        rotation = new Rotation2d(targetPoseBotSpace[5]);
        translation = new Translation2d(targetPoseBotSpace[0], targetPoseBotSpace[2]);
      }
      else {
        rotation = new Rotation2d(targetPoseCamSpace[5]);
        translation = new Translation2d(targetPoseCamSpace[0], targetPoseCamSpace[2]);
      }
    }
    return new Pose2d(translation, rotation);
  }

  public enum PoseSpace {
    ROBOT, CAMERA, TARGET;
  }

  public int getTargetID() {
    return target_id.getNumber(0).intValue();
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

  // ### Raspberry Pi ### \\

  public Translation2d getNoteScreenPose() {
    return new Translation2d(
      noteOffsetX,
      noteOffsetY
    );
  }

  public double getNoteScreenOffsetX() {
    return noteOffsetX;
  }
  public double getNoteScreenOffsetY() {
    return noteOffsetY;
  }
  public double getNoteScreenArea() {
    return noteScreenArea;
  }

  public double[] getNoteHomographyMatrix() {
    return noteHomographyMatrix;
  }

  public double[] getNotePose3D() {
    return note3dPose;
  }

  public double getNoteDistance() {
    return noteDistance;
  }

  public void configureEntries() {
    target_valid = m_limelightTable.getEntry("tv");
    target_x = m_limelightTable.getEntry("tx");
    target_y = m_limelightTable.getEntry("ty");
    target_area = m_limelightTable.getEntry("ta");

    target_id = m_limelightTable.getEntry("tid");

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

    // ### Raspberry Pi ### \\

    target_note_x = m_raspberryPiTable.getEntry("tx");
    target_note_y = m_raspberryPiTable.getEntry("ty");
    target_note_area = m_raspberryPiTable.getEntry("ta");
    target_note_homography_matrix = m_raspberryPiTable.getEntry("homography");
    target_note_estimated_3d_pose = m_raspberryPiTable.getEntry("pose");
    target_note_estimated_distance = m_raspberryPiTable.getEntry("td");
  }
}
