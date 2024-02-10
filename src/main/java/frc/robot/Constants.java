// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import frc.robot.utils.ModulePosition;
import frc.robot.utils.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
  public static class kControls {
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
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(2);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(2);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  /** All swerve constants. */
  public static class kSwerve {
    /** Constants that apply to the whole drive train. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(19.5); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(19.5); // Length of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
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
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 35;
    public static final int ANGLE_CURRENT_LIMIT = 25;

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
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 4.0;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_REVERSED = true;
    public static final boolean ANGLE_MOTOR_REVERSED = false;
    public static final boolean CAN_CODER_REVERSED = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kBrake;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kBrake;

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
      -295.04,
      ModulePosition.FRONT_LEFT
    );

    //FrontRight
    public static final SwerveModuleConstants FRONT_RIGHT_MODULE = new SwerveModuleConstants(
      4,
      5,
      0,
      -80.06,
      ModulePosition.FRONT_RIGHT
    );

    //BackLeft
    public static final SwerveModuleConstants BACK_LEFT_MODULE = new SwerveModuleConstants(
      10,
      11,
      2,
      -9.84,
      ModulePosition.BACK_LEFT
    );

    //BackRight
    public static final SwerveModuleConstants BACK_RIGHT_MODULE = new SwerveModuleConstants(
      6,
      7,
      3,
      -76.46,
      ModulePosition.BACK_RIGHT
    );
  }

  /** All vision constants */
  public static class kVision {
    // This chooses if you want to run the vision calculations on the raspberry pi or the roborio. 
    // Usually keep as true unless testing something
    public static final boolean PiVision = true;
  }

  /** All manipulator constants. */
  public static class kManip {
    public static final int INTAKE_ANGLE_MOTOR_ID = 0;
    public static final int INTAKE_SPEED_MOTOR_ID = 0;

    public static final int SHOOTER_MOTOR_A_ID = 0;
    public static final int SHOOTER_MOTOR_B_ID = 0;

    public static final double INTAKE_ANGLE_DEADZONE = 0.0;

    public static final double INTAKE_ANGLE_PID_P = 0.0;
    public static final double INTAKE_ANGLE_PID_I = 0.0;
    public static final double INTAKE_ANGLE_PID_D = 0.0;

    // First item should be how it starts and second item should be when it is down.
    public static final double[] INTAKE_ANGLE_TOGGLE_VALUES = new double[]{0, 180};
    // First item should be how it starts, second should be amp shooting, and other ones should be speaker shooting.
    public static final double[] INTAKE_SPEED_STATE_VALUES = new double[]{0, 0.3, 0.7};

    /** Default spin speed for the intake when intaking a note. */
    public static final double INTAKE_SPIN_SPEED = 0;
  }
}
