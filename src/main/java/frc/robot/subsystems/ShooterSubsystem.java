// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax shooterMotorA = new CANSparkMax(kManip.SHOOTER_MOTOR_A_ID, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax shooterMotorB = new CANSparkMax(kManip.SHOOTER_MOTOR_B_ID, CANSparkLowLevel.MotorType.kBrushless);

  /** This array stores multiple possible speeds that the shooter can be at. */
  double[] shooterSpeedStateValues = kManip.SHOOTER_SPEED_STATE_VALUES;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    configureHardware();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /** Use this to se the shooter to a custom speed. */
  public void setShooterSpeed(double speed) {
    shooterMotorA.set(speed);
    shooterMotorB.set(speed);
  }

  /** 
   * You use this to choose a state for the shooter.
   * <p>0 - Turned off
   * <p>1 - Amp Shooting
   * <p>2 - Speaker Shooting
   */
  public void setShooterState(int state) {
    setShooterSpeed(shooterSpeedStateValues[state]);
  }

  public void configureHardware() {
    shooterMotorA.restoreFactoryDefaults();
    shooterMotorA.setInverted(false);
    shooterMotorA.setIdleMode(IdleMode.kBrake);

    shooterMotorB.restoreFactoryDefaults();
    shooterMotorB.setInverted(false);
    shooterMotorB.setIdleMode(IdleMode.kBrake);
  }
}
