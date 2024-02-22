// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;
import frc.robot.utils.math.LinearInterpolator;

public class ShooterSubsystem extends SubsystemBase {

  LinearInterpolator linearInterpolator = new LinearInterpolator(kManip.SHOOTER_SPEED_ARRAY);

  CANSparkMax shooterMotorA = new CANSparkMax(kManip.SHOOTER_MOTOR_A_ID, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax shooterMotorB = new CANSparkMax(kManip.SHOOTER_MOTOR_B_ID, CANSparkLowLevel.MotorType.kBrushless);

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

  double shooterMotorASetPoint = 0.0;
  double shooterMotorBSetPoint = 0.0;

  /** This array stores multiple possible speeds that the shooter can be at. */
  double[] shooterSpeedStateValues = kManip.SHOOTER_SPEED_STATE_VALUES;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    configureHardware();
  }

  @Override
  public void periodic() {
    shooterMotorA.set(shooterMotorAPID.calculate(shooterMotorASetPoint));
    shooterMotorB.set(shooterMotorBPID.calculate(shooterMotorBSetPoint));
  }

  /** Use this to se the shooter to a custom speed. */
  public void setShooterSpeed(double speed) {
    shooterMotorASetPoint = speed;
    shooterMotorBSetPoint = speed;
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
  }

  /** 
   * This sets the speed of the shooter to match the distance you want to shoot the note. 
   * <p>This uses a Linear Interpolator based on the table kManip.SHOOTER_SPEED_ARRAY
   * @param distance the distance you want to shoot
   */
  public void setShooterDistance(double distance) {
    setShooterSpeed(linearInterpolator.getInterpolatedValue(distance));
  }

  /** This configures the motor controllers */
  public void configureHardware() {
    shooterMotorA.restoreFactoryDefaults();
    shooterMotorA.setInverted(false);
    shooterMotorA.setIdleMode(IdleMode.kBrake);

    shooterMotorB.restoreFactoryDefaults();
    shooterMotorB.setInverted(false);
    shooterMotorB.setIdleMode(IdleMode.kBrake);
  }
}
