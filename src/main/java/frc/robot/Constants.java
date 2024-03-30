// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.ModulePosition;
import frc.robot.utils.SwerveModuleConstants;
import frc.robot.utils.math.LinearInterpolator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  /** All constants for controlling the robot */
  public static class kControls {
    public static final boolean USE_LEFT_Y_FOR_INTAKING = false;

    public static final double TRANSLATION_DEADZONE = 0.1;
    public static final double ROTATION_DEADZONE = 0.05;

    public static final double AUTO_TRANSLATION_DEADZONE = 0.03;
    public static final double AUTO_ROTATION_DEADZONE = 0.03;

    public static final int DRIVE_CONTROLLER_ID = 0;
    public static final int MANIPULATOR_CONTROLLER_ID = 1;

    public static final int TRANSLATION_X_AXIS = XboxController.Axis.kLeftX.value;
    public static final int TRANSLATION_Y_AXIS = XboxController.Axis.kLeftY.value;
    public static final int ROTATION_AXIS = XboxController.Axis.kRightX.value;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(10);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(10);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(12);
  }

  /** All swerve constants. */
  public static class kSwerve {
    /** Constants that apply to the whole drive train. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(19.5); // Length of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.865);
    public static final double WHEEL_CIRCUMFRENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );

    public static final Translation2d INITIAL_TRANSLATION = new Translation2d(0, 0);
    public static final Rotation2d INITIAL_ROTATION = new Rotation2d(0);
    public static final Pose2d INITIAL_POSE = new Pose2d(INITIAL_TRANSLATION, INITIAL_ROTATION);

    public static final double DRIVE_GEAR_RATIO = 6.75; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFRENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = 12.8; // 12.8:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2);
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.05;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 50;
    public static final int ANGLE_CURRENT_LIMIT = 45;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.05;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.4;
    public static final double DRIVE_KF = 0.0;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.6335;
    public static final double DRIVE_KA = 0.46034;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.8;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.055;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 10.0;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 11.0;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_REVERSED = false;
    public static final boolean ANGLE_MOTOR_REVERSED = false;
    public static final boolean CAN_CODER_REVERSED = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

    /** 
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */

     //FrontLeft
    public static final SwerveModuleConstants FRONT_LEFT_MODULE = new SwerveModuleConstants(
      8,
      9,
      1,
      -117.07,
      ModulePosition.FRONT_LEFT
    );

    //FrontRight
    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
      4,
      5,
      0,
      -260.5,
      ModulePosition.FRONT_RIGHT
    );

    //BackLeft
    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
      10,
      11,
      2,
      -192.5,
      ModulePosition.BACK_LEFT
    );

    //BackRight
    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
      6,
      7,
      3,
      -257.1,
      ModulePosition.BACK_RIGHT
    );
  }

  /** All vision constants */
  public static class kVision {
    // This chooses if you want to run the vision calculations on the raspberry pi or the roborio. 
    // Usually keep as true unless testing something
    public static final boolean PiVision = false;
    public static final boolean detectNotes = false;

    /** This is the minimum area a contour can be in order to be detected by {@link frc.robot.utils.vision.ShapeDetection ShapeDetection} */
    public static final double MIN_CONTOUR_AREA = 0.05;

    public static final int DEFAULT_PIPELINE = 1;
    /** 
     * This is the delay {@link frc.robot.commands.LimelightDefault the limelight default command} 
     * uses to determine wether or not to blink.
     */
    public static final double TARGET_VANISH_DELAY = 2;

    /** 
     * This boolean tells {@link frc.robot.commands.SnapToAmp SnapToAmp()} 
     * and {@link frc.robot.commands.SnapToSpeaker SnapToSpeaker()} if they should use relative field position
     * as a factor in determining the angle to turn to.
     */
    public static final boolean USE_FIELD_POS_IN_SNAP_COMMANDS = false;

    //#region AprilTag IDs
    // Blue Speaker
    public static final int SPEAKER_APRILTAG_ID_BLUE = 7;
    public static final int SPEAKER_SIDE_APRILTAG_ID_BLUE = 8;
    // Red Speaker
    public static final int SPEAKER_APRILTAG_ID_RED = 3;
    public static final int SPEAKER_SIDE_APRILTAG_ID_RED = 4;
    // Blue Amp
    public static final int AMP_APRILTAG_ID_BLUE = 6;
    // Red Amp
    public static final int AMP_APRILTAG_ID_RED = 5;
    // Blue Source
    public static final int SOURCE_LEFT_APRILTAG_ID_BLUE = 2;
    public static final int SOURCE_RIGHT_APRILTAG_ID_BLUE = 1;
    // Red Source
    public static final int SOURCE_LEFT_APRILTAG_ID_RED = 10;
    public static final int SOURCE_RIGHT_APRILTAG_ID_RED = 9;
    // Blue Stage
    public static final int STAGE_CENTER_APRILTAG_ID_BLUE = 14;
    public static final int STAGE_LEFT_APRILTAG_ID_BLUE = 16;
    public static final int STAGE_RIGHT_APRILTAG_ID_BLUE = 15;
    // Red Stage
    public static final int STAGE_CENTER_APRILTAG_ID_RED = 13;
    public static final int STAGE_LEFT_APRILTAG_ID_RED = 11;
    public static final int STAGE_RIGHT_APRILTAG_ID_RED = 12;
    //#endregion
  }

  /** All manipulator constants. */
  public static class kManip {
    /** This is the amount of time that the shooter runs for when shooting a note*/
    public static final double SHOOT_TIME = 0.4;

    public static final double INTAKE_DEADZONE = 0.1;

    public static final int INTAKE_SPEED_MOTOR_A_ID = 20;
    public static final int INTAKE_SPEED_MOTOR_B_ID = 21;

    public static final int SHOOTER_MOTOR_A_ID = 15;
    public static final int SHOOTER_MOTOR_B_ID = 16;

    public static final int CLIMB_MOTOR_A_ID = 4;
    public static final int CLIMB_MOTOR_B_ID = 5;

    public static final double CLIMB_MOTOR_PID_P = 0.0;
    public static final double CLIMB_MOTOR_PID_I = 0.0;
    public static final double CLIMB_MOTOR_PID_D = 0.0;

    public static final boolean CLIMB_MOTOR_A_INVERTED = true;
    public static final boolean CLIMB_MOTOR_B_INVERTED = false;

    public static final NeutralMode CLIMB_MOTOR_NEUTRAL_MODE = NeutralMode.Brake;

    public static final int CLIMB_CURRENT_LIMIT = 80;
    public static final double CLIMB_VOLTAGE_RAMP = 0;

    public static final double CLIMB_ROTATION_TO_HEIGHT = 1/360;
    public static final double CLIMB_HEIGHT_TO_ROTATION = 1/CLIMB_ROTATION_TO_HEIGHT;

    public static final double MINIMUM_CLIMB_HEIGHT = 0;
    public static final double MAXIMUM_CLIMB_HEIGHT = 1.0;

    public static final double SHOOTER_MOTOR_A_PID_P = 0.0;
    public static final double SHOOTER_MOTOR_A_PID_I = 0.0;
    public static final double SHOOTER_MOTOR_A_PID_D = 0.0;

    public static final double SHOOTER_MOTOR_B_PID_P = 0.0;
    public static final double SHOOTER_MOTOR_B_PID_I = 0.0;
    public static final double SHOOTER_MOTOR_B_PID_D = 0.0;

    // 0.225 for amp (battery dead :( ))
    // First item should be how it starts, second should be amp shooting, and other ones should be speaker shooting.
    public static final double[] SHOOTER_SPEED_STATE_VALUES = new double[]{
      0, 0.225, 1.0
    };

    public static final boolean USE_TUNED_SHOOTER_VALUES = false;

    /** Default spin speed for the intake when intaking a note. */
    public static final double INTAKE_SPIN_SPEED = -0.6;

    /** Default spin speed for the indexer when intaking a note. */
    public static final double INDEXER_SPIN_SPEED = -0.8;

    /** The delay that will happen in between the beam break sensor triggering and the intake stopping */
    public static double BEAM_BREAK_SENSOR_INDEXER_DELAY = 0.0;

    /** Table of shooter spin speeds and the distance they shoot. <p>TODO: Make the table */
    public static final double[][] SHOOTER_SPEED_ARRAY = {
      {0, 0},
    };

    /**
     * This is used to linearly interpolate between the values collected by 
     * {@link frc.robot.Constants.kManip#SHOOTER_SPEED_ARRAY this array} 
     */
    public static final LinearInterpolator SHOOTER_LINEAR_INTERPOLATOR = new LinearInterpolator(SHOOTER_SPEED_ARRAY);

    public static final double VISION_SHOOTING_LINEAR_MUL = 1.0;
    public static final double VISION_SHOOTING_EXPONENT = 1.0;
    public static final double VISION_SHOOTING_OFFSET = 0.0;

    public static final double MAX_SHOOTING_DISTANCE_SPEAKER = 1.8;
  }
  public static class kNetworkTables {
    /** This is the name of the main networktables table for the robot. */
    public static final String MAIN_TABLE_NAME = "robot";
  }

  public static class kField {
    /** The X position of the red speaker in field space */
    public static final double RED_SPEAKER_X = 0.0;
    /** The Z position of the red speaker in field space */
    public static final double RED_SPEAKER_Z = 0.0;

    /** The X position of the blue speaker in field space */
    public static final double BLUE_SPEAKER_X = 0.0;
    /** The Z position of the blue speaker in field space */
    public static final double BLUE_SPEAKER_Z = 0.0;

    /** The X position of the red amp in field space */
    public static final double RED_AMP_X = 0.0;
    /** The Z position of the red amp in field space */
    public static final double RED_AMP_Z = 0.0;

    /** The X position of the blue amp in field space */
    public static final double BLUE_AMP_X = 0.0;
    /** The Z position of the blue amp in field space */
    public static final double BLUE_AMP_Z = 0.0;
  }

  /** Constants For Camera Displays */
  public static class kDisplay {
    public static final int RENDER_POINT_RADIUS = 2;
    public static final int NAME_RENDER_SIZE = 2;
    public static final Point NAME_RENDER_OFFSET = new Point(0, 0);
    public static final int LINE_THICKNESS = 1;

    public static final Scalar LINE_COLOR = new Scalar(255, 255, 0);
    public static final Scalar DETAIL_COLOR = new Scalar(255, 100, 0);

    public static final Point FPS_COUNTER_POSITION = new Point(100, 100);
    public static final double FPS_COUNTER_SIZE = 10.0;
    public static final Scalar FPS_COUNTER_COLOR = new Scalar(0, 0, 0);

    public static final Point NOTE_COUNTER_POSITION = new Point(100, 100);
    public static final double NOTE_COUNTER_SIZE = 10.0;
    public static final Scalar NOTE_COUNTER_COLOR = new Scalar(0, 0, 0);
  }

  public static class kLED {
    public static final int STRIP_LENGTH = 99;
  }
}
