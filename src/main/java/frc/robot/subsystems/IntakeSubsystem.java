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
    intakeSpinMotorA.set(speed);
    intakeSpinMotorB.set(speed);
  }

  /** This configures the motor controllers */
  private void configureHardware() {
    intakeSpinMotorA.restoreFactoryDefaults();
    intakeSpinMotorA.setInverted(false);
    intakeSpinMotorA.setIdleMode(IdleMode.kBrake);

    intakeSpinMotorB.restoreFactoryDefaults();
    intakeSpinMotorB.setInverted(false);
    intakeSpinMotorB.setIdleMode(IdleMode.kBrake);
  }
}
