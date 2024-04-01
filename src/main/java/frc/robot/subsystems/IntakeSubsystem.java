// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;

public class IntakeSubsystem extends SubsystemBase {

  boolean topReversed = false;
  boolean bottomReversed = false;

  CANSparkMax intakeSpinMotorA = new CANSparkMax(kManip.INTAKE_SPEED_MOTOR_A_ID, CANSparkLowLevel.MotorType.kBrushless);
  CANSparkMax intakeSpinMotorB = new CANSparkMax(kManip.INTAKE_SPEED_MOTOR_B_ID, CANSparkLowLevel.MotorType.kBrushless);

  /** Creates a new ManipulatorSubsystem. */
  public IntakeSubsystem() {
    configureHardware();
  }

  @Override
  public void periodic() {

  }

  /** Use this to set the intake to a custom speed. */
  public void setIntakeSpinSpeed(double speed) {
    if(topReversed) {
      intakeSpinMotorA.set(-speed);
    } else {
      intakeSpinMotorA.set(speed);
    }
    if(bottomReversed) {
      intakeSpinMotorB.set(-speed);
    } else {
      intakeSpinMotorB.set(speed);
    }
  }

  public void toggleTop() {
    if(topReversed) {
      topReversed = false;
    } else {
      topReversed = true;
    }
  }
  public void toggleBottom() {
    if(bottomReversed) {
      bottomReversed = false;
    } else {
      bottomReversed = true;
    }
  }

  /** This configures the motor controllers */
  private void configureHardware() {
    intakeSpinMotorA.restoreFactoryDefaults();
    intakeSpinMotorA.setInverted(true);
    intakeSpinMotorA.setIdleMode(IdleMode.kCoast);
    intakeSpinMotorA.setSmartCurrentLimit(20);
    //intakeSpinMotorA.setOpenLoopRampRate(0.25);
    //intakeSpinMotorA.setClosedLoopRampRate(0.25);

    intakeSpinMotorB.restoreFactoryDefaults();
    intakeSpinMotorB.setInverted(true);
    intakeSpinMotorB.setIdleMode(IdleMode.kCoast);
    intakeSpinMotorB.setSmartCurrentLimit(20);
    //intakeSpinMotorB.setOpenLoopRampRate(0.25);
    //intakeSpinMotorB.setClosedLoopRampRate(0.25);
  }
}
