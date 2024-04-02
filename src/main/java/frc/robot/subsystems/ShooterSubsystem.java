// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;
import frc.robot.utils.debug.TunableNumber;
import frc.robot.utils.math.LinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {

  LinearInterpolator linearInterpolator = new LinearInterpolator(kManip.SHOOTER_SPEED_ARRAY);

  TunableNumber speakerShootSpeed = new TunableNumber("Shooter Speed: Speaker");
  TunableNumber ampShootSpeed = new TunableNumber("Shooter Speed: Amp");

  CANSparkMax shooterMotorA = new CANSparkMax(kManip.SHOOTER_MOTOR_A_ID, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax shooterMotorB = new CANSparkMax(kManip.SHOOTER_MOTOR_B_ID, CANSparkLowLevel.MotorType.kBrushless);

  CANSparkMax ampModuleMotor = new CANSparkMax(kManip.AMP_MODULE_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

  RelativeEncoder ampModuleEncoder = ampModuleMotor.getEncoder();

  PIDController shooterMotorAPID = new PIDController(
    kManip.SHOOTER_MOTOR_A_PID_P, 
    kManip.SHOOTER_MOTOR_A_PID_I, 
    kManip.SHOOTER_MOTOR_A_PID_D
  );
  PIDController shooterMotorBPID = new PIDController(
    kManip.SHOOTER_MOTOR_B_PID_P, 
    kManip.SHOOTER_MOTOR_B_PID_I, 
    kManip.SHOOTER_MOTOR_B_PID_D
  );

  PIDController ampModulePID = new PIDController(
    kManip.AMP_MODULE_P,
    kManip.AMP_MODULE_I,
    kManip.AMP_MODULE_D
  );

  double shooterMotorASetPoint = 0.0;
  double shooterMotorBSetPoint = 0.0;

  double ampModuleSetpoint = 0.0;

  double ampModuleSpeed = 0.0;

  /** This array stores multiple possible speeds that the shooter can be at. */
  double[] shooterSpeedStateValues = kManip.SHOOTER_SPEED_STATE_VALUES;

  boolean usePID = true;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    configureHardware();
    speakerShootSpeed.setDefault(0.7);
    ampShootSpeed.setDefault(0.3);
  }

  @Override
  public void periodic() {
    // shooterMotorA.set(shooterMotorAPID.calculate(shooterMotorASetPoint));
    // shooterMotorB.set(shooterMotorBPID.calculate(shooterMotorBSetPoint));

    shooterMotorA.set(shooterMotorASetPoint);
    shooterMotorB.set(shooterMotorBSetPoint);

    if(kManip.USE_TUNED_SHOOTER_VALUES) {
      // shooterSpeedStateValues[2] = speakerShootSpeed.get();
      // shooterSpeedStateValues[1] = ampShootSpeed.get();
    } else {
      // shooterSpeedStateValues = kManip.SHOOTER_SPEED_STATE_VALUES;
    }

    SmartDashboard.putNumber("Amp Module Encoder Position", ampModuleEncoder.getPosition());
    if(usePID) {
      ampModuleMotor.set(ampModulePID.calculate(ampModuleEncoder.getPosition(), ampModuleSetpoint));
    } else {
      ampModuleMotor.set(ampModuleSpeed);
    }
  }

  /** Use this to se the shooter to a custom speed. */
  public void setShooterSpeed(double speed) {
    shooterMotorASetPoint = speed;
    shooterMotorBSetPoint = speed;
  }

  public double getShooterSpeed() {
    return shooterMotorA.getOutputCurrent();
  }

  /** 
   * You use this to choose a state for the shooter.
   * @param state
   * <p>0 - Turned off
   * <p>1 - Amp Shooting
   * <p>2 - Speaker Shooting
   */
  public void setShooterState(int state) {
    setShooterSpeed(shooterSpeedStateValues[state]);
    if(usePID) {
      switch (state) {
        case 1:
          setAmpModulePosition(kManip.AMP_MODULE_RAISED_ROTATION);
          break;
        case 2:
          setAmpModulePosition(kManip.AMP_MODULE_LOWERED_ROTATION);
          break;
      
        default:
          break;
      }
    } else {
      if(state == 1) {
        setAmpModulePosition(0.1);
      } else {
        setAmpModulePosition(-0.1);
      }
    }
  }

  /** 
   * This sets the speed of the shooter to match the distance you want to shoot the note. 
   * <p>This uses a Linear Interpolator based on the table kManip.SHOOTER_SPEED_ARRAY
   * @param distance the distance you want to shoot
   */
  public void setShooterDistance(double distance) {
    setShooterSpeed(linearInterpolator.getInterpolatedValue(distance));
  }

  /** 
   * This sets the speed of the shooter to match the distance you want to shoot the note. 
   * <p>This uses a Linear Interpolator based on the table kManip.SHOOTER_SPEED_ARRAY
   * @param distance the distance you want to shoot
   * @param linearMul this multiplies the final speed by a value
   * @param exponentialMul this takes the final speed to an exponent speed^exponentialMul
   * @param speedOffset this adds a certain amount to the final speed
   */
  public void setShooterDistance(double distance, double linearMul, double exponent, double speedOffset) {
    double finalSpeed = linearInterpolator.getInterpolatedValue(distance);
    finalSpeed *= linearMul;
    finalSpeed = Math.pow(finalSpeed, exponent);
    finalSpeed += speedOffset;
    setShooterSpeed(finalSpeed);
  }

  public void setAmpModulePosition(double position) {
    if(usePID) {
      ampModuleSetpoint = position;
    } else {
      ampModuleSpeed = position;
    }
  }

  /** This configures the motor controllers */
  public void configureHardware() {
    shooterMotorA.restoreFactoryDefaults();
    shooterMotorA.setInverted(false);
    shooterMotorA.setIdleMode(IdleMode.kBrake);
    shooterMotorA.setSmartCurrentLimit(kManip.SHOOTER_CURRENT_LIMIT);
    shooterMotorA.setOpenLoopRampRate(kManip.SHOOTER_VOLTAGE_RAMP);
    shooterMotorA.setClosedLoopRampRate(kManip.SHOOTER_VOLTAGE_RAMP);

    shooterMotorB.restoreFactoryDefaults();
    shooterMotorB.setInverted(true);
    shooterMotorB.setIdleMode(IdleMode.kBrake);
    shooterMotorB.setSmartCurrentLimit(kManip.SHOOTER_CURRENT_LIMIT);
    shooterMotorB.setOpenLoopRampRate(kManip.SHOOTER_VOLTAGE_RAMP);
    shooterMotorB.setClosedLoopRampRate(kManip.SHOOTER_VOLTAGE_RAMP);

    ampModuleMotor.restoreFactoryDefaults();
    ampModuleMotor.setInverted(false);
    ampModuleMotor.setIdleMode(IdleMode.kBrake);
    ampModuleMotor.setSmartCurrentLimit(kManip.AMP_MODULE_CURRENT_LIMIT);
    ampModuleMotor.setOpenLoopRampRate(kManip.AMP_MODULE_VOLTAGE_RAMP);
    ampModuleMotor.setClosedLoopRampRate(kManip.AMP_MODULE_VOLTAGE_RAMP);

    ampModuleEncoder = ampModuleMotor.getEncoder();
    ampModuleEncoder.setPosition(0.0);
  }
}
