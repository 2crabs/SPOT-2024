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

  /** 
   * Sets the limelight's LED mode
   * @param pipelineNumber The LED mode
  */
  public void setLEDMode(LimelightLEDMode mode) {
    led_mode.setNumber(mode.modeID);
  }

  /** 
   * Gets the current LED mode of the limelight
   * @return The LED mode
   */
  public LimelightLEDMode getLEDMode() {
    return LimelightLEDMode.fromModeID(led_mode.getNumber(1).intValue());
  }

  /** 
   * Gets the current pipeline of the limelight
   * @return The pipeline number as an int
   */
  public int getCurrentPipeline() {
    return pipeline_get.getNumber(0).intValue();
  }

  /** 
   * Sets the limelight's pipeline
   * @param pipelineNumber The pipeline number as an int
  */
  public void setPipeline(int pipelineNumber) {
    pipeline_set.setNumber(pipelineNumber);
  }

  /** Returns true if the limelight has a valid target apriltag */
  public boolean hasValidTarget() {
    return target_valid.getDouble(0) != 0.0;
  }

  /** The latency from the limelight */
  public double getLatencyMS() {
    return pipeline_latency.getDouble(0) + image_latency.getDouble(0.0);
  }

  /** The X offset of the targeted apriltag (54 degree range) */
  public double getTargetOffsetX() {
    return targetOffsetX;
  }
  /** The Y offset of the targeted apriltag */
  public double getTargetOffsetY() {
    return targetOffsetY;
  }
  /** The area of the targeted apriltag */
  public double getTargetArea() {
    return targetArea;
  }
  /** The skew of the targeted apriltag */
  public double getTargetSkew() {
    return targetSkew;
  }

  /** The length of the shortest side on the targeted apriltag in pixels */
  public double getTargetShortSideLength() {
    return short_side_length.getDouble(0);
  }
  /** The length of the longest side on the targeted apriltag in pixels */
  public double getTargetLongSideLength() {
    return long_side_length.getDouble(0);
  }
  /** The length of the x side on the targeted apriltag in pixels */
  public double getTargetXSideLength() {
    return x_side_length.getDouble(0);
  }
  /** The length of the y side on the targeted apriltag in pixels */
  public double getTargetYSideLength() {
    return y_side_length.getDouble(0);
  }

  /**
   * This is the position of the robot on the field
   * @return A {@link edu.wpi.first.math.geometry.Pose2d Pose2d} contatining the position of the robot
   */
  public Pose2d getBotPose() {
    Rotation2d rotation = new Rotation2d(botPose[5]);
    Translation2d translation = new Translation2d(botPose[0],botPose[2]);
    return new Pose2d(translation, rotation);
  }

  /**
   * This is the position of the robot on the field based on the red alliance.
   * @return A {@link edu.wpi.first.math.geometry.Pose2d Pose2d} contatining the position of the robot
   */
  public Pose2d getBotPoseRed() {
    Rotation2d rotation = new Rotation2d(botPoseRed[5]);
    Translation2d translation = new Translation2d(botPoseRed[0],botPoseRed[2]);
    return new Pose2d(translation, rotation);
  }

  /**
   * This is the position of the robot on the field based on the blue alliance.
   * @return A {@link edu.wpi.first.math.geometry.Pose2d Pose2d} contatining the position of the robot
   */
  public Pose2d getBotPoseBlue() {
    Rotation2d rotation = new Rotation2d(botPoseBlue[5]);
    Translation2d translation = new Translation2d(botPoseBlue[0],botPoseBlue[2]);
    return new Pose2d(translation, rotation);
  }

  /**
   * This is the pose _x_ in _y_ space, based on vision measurements of the limelight.
   * @param get A {@link frc.robot.subsystems.Vision.PoseSpace PoseEnum} of the space you want to return.
   * @param space A {@link frc.robot.subsystems.Vision.PoseSpace PoseEnum} of the space you want to return.
   * @return The position of [get] in [space] space.
   */
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

  /**
   * The ID of the targeted april tag.
   * @return The ID as an integer.
   */
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

  /**
   * This returns the x and y offset of the note
   * @return A {@link edu.wpi.first.math.geometry.Translation2d Translation2d} 
   * containig the position of the note in pixels
   */
  public Translation2d getNoteScreenPose() {
    return new Translation2d(
      noteOffsetX,
      noteOffsetY
    );
  }

  /**
   * This returns the x offset of the note based on the center
   * @return The x offset of the note in pixels
   */
  public double getNoteScreenOffsetX() {
    return noteOffsetX;
  }
  /**
   * This returns the y offset of the note based on the center
   * @return The y offset of the note in pixels
   */
  public double getNoteScreenOffsetY() {
    return noteOffsetY;
  }
  /**
   * This returns the area of the target note
   * @return The area of the note in pixels
   */
  public double getNoteScreenArea() {
    return noteScreenArea;
  }

  /**
   * This returns the homography matrix estimated by the raspberry pi.
   * @return A double array containing all values in the homography matrix
   */
  public double[] getNoteHomographyMatrix() {
    return noteHomographyMatrix;
  }

  /** 
   * This returns the estimated 3d pose of the raspberry pi's target note
   * @return A double array containing the 3d pose of the note in camera space
   * <p> getNotePos3D()[0] = X position
   * <p> getNotePos3D()[1] = Y position
   * <p> getNotePos3D()[2] = Z position
   */
  public double[] getNotePose3D() {
    return note3dPose;
  }

  /** 
   * This returns the distance that the raspberry pi belives the note to be from the camera
   * @return The distance from the target note to the robot's camera
   */
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
