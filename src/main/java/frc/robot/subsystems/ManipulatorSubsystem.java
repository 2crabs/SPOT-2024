// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;

public class ManipulatorSubsystem extends SubsystemBase {

  TalonSRX intakeAngleMotor = new TalonSRX(kManip.INTAKE_ANGLE_MOTOR_ID);

  CANSparkMax intakeSpinMotor = new CANSparkMax(kManip.INTAKE_SPEED_MOTOR_ID, null);

  CANSparkMax shooterMotorA = new CANSparkMax(kManip.SHOOTER_MOTOR_A_ID, null);
  CANSparkMax shooterMotorB = new CANSparkMax(kManip.SHOOTER_MOTOR_B_ID, null);

  PIDController intakeAnglePID = new PIDController(kManip.INTAKE_ANGLE_PID_P, kManip.INTAKE_ANGLE_PID_I, kManip.INTAKE_ANGLE_PID_D);

  /** This is the angle we use in the PID as the setpoint. */
  double intakeTargetAngle = 0;

  /**
   * This array stores multiple possible angles that the intake can be at. this should only be 0 and 180, but we can add more values
   * to the list later.
   */
  double[] intakeAngleToggleValues = kManip.INTAKE_ANGLE_TOGGLE_VALUES;

  /** This array stores multiple possible speeds that the shooter can be at. */
  double[] shooterSpeedStateValues = kManip.INTAKE_SPEED_STATE_VALUES;

  /** Creates a new ManipulatorSubsystem. */
  public ManipulatorSubsystem() {
    configureHardware();
    intakeTargetAngle = intakeAngleToggleValues[0];
  }

  @Override
  public void periodic() {
    double pidCalculation = intakeAnglePID.calculate(getIntakeAngle(), intakeTargetAngle);
    // Applies a deadzone to the speed.
    if(pidCalculation < kManip.INTAKE_ANGLE_DEADZONE) {
      intakeAngleMotor.set(TalonSRXControlMode.PercentOutput, pidCalculation);
    } else {
      intakeAngleMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
  }

  /** Sets the target angle for the intake (degrees) */
  public void setIntakeMotorAngle(double angle) {
    intakeTargetAngle = angle;
  }

  /**
   * This command shuffles through all of the values in the list of possible angles for the intake.
   */
  public void switchIntakeMotorAngle() {
    boolean isEqual = false;
    for(int i = 0; i < intakeAngleToggleValues.length-1; i ++) {
      if(intakeTargetAngle == intakeAngleToggleValues[i]) {
        intakeTargetAngle = intakeAngleToggleValues[i+1];
        isEqual = true;
      }
    }
    if (!isEqual) {
      intakeTargetAngle = intakeAngleToggleValues[0];
    }
  }

  /** 
   * You use this to set the angle of the intake motor to one of the predetermined values in the 
   * array "intakeAngleToggleValues" 
  */
  public void setPredeterminedIntakeMotorAngle(int index) {
    intakeTargetAngle = intakeAngleToggleValues[index];
  }

  /** Use this to set the intake to a custom speed. */
  public void setIntakeSpinSpeed(double speed) {
    intakeSpinMotor.set(speed);
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

  /** @return The angle the PID controller for the intake angle motor is trying to go to. */
  public double getIntakeTargetAngle() {
    return 0; // Not Finished
  }

  /** @return The current angle of the intake */
  public double getIntakeAngle() {
    return 0; // Not Finished
  }

  private void configureHardware() {
    intakeAngleMotor.setInverted(false);
    intakeAngleMotor.setNeutralMode(NeutralMode.Brake);

    intakeSpinMotor.restoreFactoryDefaults();
    intakeSpinMotor.setInverted(false);
    intakeSpinMotor.setIdleMode(IdleMode.kBrake);

    shooterMotorA.restoreFactoryDefaults();
    shooterMotorA.setInverted(false);
    shooterMotorA.setIdleMode(IdleMode.kBrake);

    shooterMotorB.restoreFactoryDefaults();
    shooterMotorB.setInverted(false);
    shooterMotorB.setIdleMode(IdleMode.kBrake);
  }
}
