// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;
import frc.robot.utils.debug.TunableNumber;

public class IntakeSubsystem extends SubsystemBase {

  private boolean angleMotorUsingPID = false;

  TunableNumber intakeUpAngle = new TunableNumber("Intake Up Angle");
  TunableNumber intakeDownAngle = new TunableNumber("Intake Down Angle");

  TalonSRX intakeAngleMotor = new TalonSRX(kManip.INTAKE_ANGLE_MOTOR_ID);

  CANSparkMax intakeSpinMotor = new CANSparkMax(kManip.INTAKE_SPEED_MOTOR_ID, CANSparkLowLevel.MotorType.kBrushless);

  TunableNumber intakeAnglePID_P = new TunableNumber("Intake Angle PID: P");
  TunableNumber intakeAnglePID_I = new TunableNumber("Intake Angle PID: I");
  TunableNumber intakeAnglePID_D = new TunableNumber("Intake Angle PID: D");

  PIDController intakeAnglePID = new PIDController(kManip.INTAKE_ANGLE_PID_P, kManip.INTAKE_ANGLE_PID_I, kManip.INTAKE_ANGLE_PID_D);

  /** This is the angle we use in the PID as the setpoint. */
  double intakeTargetAngle = 0;

  double intakeTargetSpeed = 0.0;

  /**
   * This array stores multiple possible angles that the intake can be at. this should only be 0 and 180, but we can add more values
   * to the list later.
   */
  double[] intakeAngleToggleValues = kManip.INTAKE_ANGLE_TOGGLE_VALUES;

  /** Creates a new ManipulatorSubsystem. */
  public IntakeSubsystem() {
    configureHardware();
    intakeTargetAngle = intakeAngleToggleValues[0];

    intakeUpAngle.setDefault(0);
    intakeDownAngle.setDefault(45);

    intakeAnglePID_P.setDefault(kManip.INTAKE_ANGLE_PID_P);
    intakeAnglePID_I.setDefault(kManip.INTAKE_ANGLE_PID_I);
    intakeAnglePID_D.setDefault(kManip.INTAKE_ANGLE_PID_D);
  }

  @Override
  public void periodic() {
    if(kManip.USE_TUNED_INTAKE_VALUES) {
      intakeAngleToggleValues[0] = intakeUpAngle.get();
      intakeAngleToggleValues[1] = intakeDownAngle.get();

      intakeAnglePID.setPID(
        intakeAnglePID_P.get(),
        intakeAnglePID_I.get(),
        intakeAnglePID_D.get()
      );
    }

    if(angleMotorUsingPID) {
      double pidCalculation = intakeAnglePID.calculate(getIntakeAngle(), intakeTargetAngle);
      // Applies a deadzone to the speed.
      if(pidCalculation > kManip.INTAKE_ANGLE_DEADZONE) {
        intakeAngleMotor.set(TalonSRXControlMode.PercentOutput, pidCalculation);
      } else {
        intakeAngleMotor.set(TalonSRXControlMode.PercentOutput, 0);
      }
    }
    else {
      if(Math.abs(intakeTargetSpeed) > kManip.INTAKE_ANGLE_DEADZONE) {
        intakeAngleMotor.set(TalonSRXControlMode.PercentOutput, intakeTargetSpeed);
      } else {
        intakeAngleMotor.set(TalonSRXControlMode.PercentOutput, 0);
      }
    }

    SmartDashboard.putNumber("intakeAngle", intakeAngleMotor.getSelectedSensorPosition());
  }

  /** Sets the target angle for the intake (degrees) */
  public void setIntakeMotorAngle(double angle) {
    angleMotorUsingPID = true;
    intakeTargetAngle = angle;
  }

  public void setIntakeMotorAngleSpeed(double speed) {
    angleMotorUsingPID = false;
    intakeTargetSpeed = speed;
  }

  /**
   * This command shuffles through all of the values in the list of possible angles for the intake.
   */
  public void switchIntakeMotorAngle() {
    boolean isEqual = false;
    for(int i = 0; i < intakeAngleToggleValues.length-1; i ++) {
      if(intakeTargetAngle == intakeAngleToggleValues[i]) {
        setIntakeMotorAngle(intakeAngleToggleValues[i+1]);
        isEqual = true;
      }
    }
    if (!isEqual) {
      setIntakeMotorAngle(intakeAngleToggleValues[0]);
    }
  }

  /** 
   * You use this to set the angle of the intake motor to one of the predetermined values in the 
   * array "intakeAngleToggleValues" 
  */
  public void setPredeterminedIntakeMotorAngle(int index) {
    setIntakeMotorAngle(intakeAngleToggleValues[index]);
  }

  /** Use this to set the intake to a custom speed. */
  public void setIntakeSpinSpeed(double speed) {
    intakeSpinMotor.set(speed);
  }

  /** @return The angle the PID controller for the intake angle motor is trying to go to. */
  public double getIntakeTargetAngle() {
    return intakeTargetAngle;
  }

  /** @return The current angle of the intake */
  public double getIntakeAngle() {
    return intakeAngleMotor.getSelectedSensorPosition();
  }

  /** This configures the motor controllers */
  private void configureHardware() {
    intakeAngleMotor.setInverted(false);
    intakeAngleMotor.setNeutralMode(NeutralMode.Brake);

    intakeSpinMotor.restoreFactoryDefaults();
    intakeSpinMotor.setInverted(false);
    intakeSpinMotor.setIdleMode(IdleMode.kBrake);
  }
}
