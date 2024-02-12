package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * A single swerve module. This class handles the pid controllers and encoders for the swerve Module.
 */
public class SwerveModule {

  private final CANSparkMax wheelMotor;
  private final RelativeEncoder wheelEncoder;
  private final SparkPIDController wheelPID;
  private final SimpleMotorFeedforward wheelFeedforward;

  private final CANSparkMax turnMotor;
  /**
   * Integrated encoder for the angle motor. When getPosition is called it returns <b>the rotation of the module</b> (not the actual motor)
   */
  private final RelativeEncoder turnEncoder;
  private final SparkPIDController turnPID;
  
  private final CANcoder turnCANCoder;
  private final double CANCoderOffsetDegrees;

  /**
   * @param moduleConstants The constants used to set up the device ids on the can bus. These can be changed in {@link Constants}
   */
  public SwerveModule(SwerveModuleConstants moduleConstants) {
    
    turnMotor = new CANSparkMax(moduleConstants.turnMotorID, CANSparkLowLevel.MotorType.kBrushless);
    wheelMotor = new CANSparkMax(moduleConstants.wheelMotorID, CANSparkLowLevel.MotorType.kBrushless);
    
    wheelPID = wheelMotor.getPIDController();
    turnPID = turnMotor.getPIDController();

    wheelEncoder = wheelMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    turnCANCoder = new CANcoder(moduleConstants.canCoderID);
    CANCoderOffsetDegrees = moduleConstants.canCoderOffsetDegrees;

    wheelFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    configureDevices();

  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    //converts angle to the nearest angle of module(if opposite, the speed will be reversed)
    state = SwerveModuleState.optimize(state, getState().angle);

    //if openloop it will just set power
    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      wheelPID.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    } else {
      wheelPID.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, wheelFeedforward.calculate(state.speedMetersPerSecond));
    }
    //sets pid target for the rotation of the module
    turnPID.setReference(state.angle.getRotations(), CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber(String.valueOf(CANCoderOffsetDegrees) + " Angle With Offset Accounted For", (turnCANCoder.getAbsolutePosition().getValueAsDouble() * 360) + CANCoderOffsetDegrees);
  }

  public SwerveModuleState getState() {
    SmartDashboard.putNumber(String.valueOf(CANCoderOffsetDegrees), turnCANCoder.getAbsolutePosition().getValueAsDouble() * 360);
    double velocity = wheelEncoder.getVelocity();
    Rotation2d angle = new Rotation2d(turnEncoder.getPosition() * Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
    return new SwerveModuleState(velocity, angle);
  }

  public double getCANCoderPosition() {
    return turnCANCoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * Returns the angle of the turn module
   * @return Angle of module
   */
  public Rotation2d getAngle() {
    return new Rotation2d(turnEncoder.getPosition() * Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
  }

  public SwerveModulePosition getPosition() {
    double distance = wheelEncoder.getPosition();
    Rotation2d rot = getAngle();
    return new SwerveModulePosition(distance, rot);
  }
  
  private void configureDevices() {
    // Drive motor configuration.
    wheelMotor.restoreFactoryDefaults();
    wheelMotor.setInverted(Constants.kSwerve.DRIVE_MOTOR_REVERSED);
    wheelMotor.setIdleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    wheelMotor.setOpenLoopRampRate(Constants.kSwerve.OPEN_LOOP_RAMP);
    wheelMotor.setClosedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
    wheelMotor.setSmartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);
 
    wheelPID.setP(Constants.kSwerve.DRIVE_KP);
    wheelPID.setI(Constants.kSwerve.DRIVE_KI);
    wheelPID.setD(Constants.kSwerve.DRIVE_KD);
    wheelPID.setFF(Constants.kSwerve.DRIVE_KF);
 
    wheelEncoder.setPositionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
    wheelEncoder.setVelocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    wheelEncoder.setPosition(0);

    // Angle motor configuration.
    turnMotor.restoreFactoryDefaults();
    turnMotor.setInverted(Constants.kSwerve.ANGLE_MOTOR_REVERSED);
    turnMotor.setIdleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    turnMotor.setSmartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

    turnPID.setP(Constants.kSwerve.ANGLE_KP);
    turnPID.setI(Constants.kSwerve.ANGLE_KI);
    turnPID.setD(Constants.kSwerve.ANGLE_KD);
    turnPID.setFF(Constants.kSwerve.ANGLE_KF);

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMaxInput(1.0);
    turnPID.setPositionPIDWrappingMinInput(0);

    // PIDS
    turnPID.setReference(0, ControlType.kPosition);
    wheelPID.setReference(0, ControlType.kDutyCycle);

    // CanCoder configuration.

    MagnetSensorConfigs CANcoderConfig = new MagnetSensorConfigs()
      .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
    turnCANCoder.getConfigurator().apply(CANcoderConfig);

    /*
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = Constants.kSwerve.CAN_CODER_REVERSED;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
    
    turnCANCoder.configFactoryDefault();
    canCoderConfiguration.
    turnCANCoder.configAllSettings(canCoderConfiguration);
    */

    turnEncoder.setPositionConversionFactor(1/Constants.kSwerve.ANGLE_GEAR_RATIO);
    turnEncoder.setVelocityConversionFactor(1/Constants.kSwerve.ANGLE_GEAR_RATIO);
    turnEncoder.setPosition(((turnCANCoder.getAbsolutePosition().getValueAsDouble()*360.0) + CANCoderOffsetDegrees)/360.0);
  }
}