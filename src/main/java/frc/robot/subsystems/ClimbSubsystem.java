// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;
import frc.robot.utils.math.MovingAverage;

public class ClimbSubsystem extends SubsystemBase {

  private MovingAverage motorACurrent = new MovingAverage(100);
  private MovingAverage motorBCurrent = new MovingAverage(100);

  private boolean motorAAtBound, motorBAtBound = false;

  private TalonSRX climbMotorA = new TalonSRX(kManip.CLIMB_MOTOR_A_ID);
  private TalonSRX climbMotorB = new TalonSRX(kManip.CLIMB_MOTOR_B_ID);

  private PIDController climbMotorAPID = new PIDController(
    kManip.CLIMB_MOTOR_PID_P, 
    kManip.CLIMB_MOTOR_PID_I, 
    kManip.CLIMB_MOTOR_PID_D
  );

  private PIDController climbMotorBPID = new PIDController(
    kManip.CLIMB_MOTOR_PID_P, 
    kManip.CLIMB_MOTOR_PID_I, 
    kManip.CLIMB_MOTOR_PID_D
  );

  private double targetPosition = 0.0;

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    configureHardware();
  }

  @Override
  public void periodic() {
    /*
    double pidCalculation = climbMotorAPID.calculate(climbMotorA.getSelectedSensorPosition(), targetPosition);
    climbMotorA.set(ControlMode.PercentOutput, pidCalculation);
    pidCalculation = climbMotorBPID.calculate(climbMotorB.getSelectedSensorPosition(), targetPosition);
    climbMotorB.set(ControlMode.PercentOutput, pidCalculation);
    */
    SmartDashboard.putBoolean("climb A At Lower or Upper Bound", motorAAtBound);
    SmartDashboard.putBoolean("climb B At Lower or Upper Bound", motorBAtBound);

    SmartDashboard.putNumber("climb A Average Current", motorACurrent.getAverage());
    SmartDashboard.putNumber("climb B Average Current", motorBCurrent.getAverage());

    SmartDashboard.putNumber("climb A Current", climbMotorA.getStatorCurrent());
    SmartDashboard.putNumber("climb B Current", climbMotorB.getStatorCurrent());

    if(Math.abs(climbMotorA.getStatorCurrent()) > 0) {
      motorACurrent.addNumber(climbMotorA.getStatorCurrent());
    }
    if(Math.abs(climbMotorB.getStatorCurrent()) > 0) {
      motorBCurrent.addNumber(climbMotorB.getStatorCurrent());
    }

    motorAAtBound = false;
    motorBAtBound = false;
    if (Math.abs(climbMotorA.getStatorCurrent()) >= Math.abs(motorACurrent.getAverage()) + 5) {
      motorAAtBound = true;
    }
    if (Math.abs(climbMotorB.getStatorCurrent()) >= Math.abs(motorBCurrent.getAverage()) + 5) {
      motorBAtBound = true;
    }
  }

  public void setClimbSpeed(double speed) {
    if(!motorAAtBound) {
      climbMotorA.set(ControlMode.PercentOutput, speed);
      climbMotorB.set(ControlMode.PercentOutput, speed);
    }
  }

  public void setClimbSpeedA(double speed) {
    if(!motorAAtBound) {
      climbMotorA.set(ControlMode.PercentOutput, speed);
    }
  }
  public void setClimbSpeedB(double speed) {
    if(!motorBAtBound) {
      climbMotorB.set(ControlMode.PercentOutput, speed);
    }
  }

  private void configureHardware() {
    TalonSRXConfiguration configA = new TalonSRXConfiguration();
    TalonSRXConfiguration configB = new TalonSRXConfiguration();

    configA.closedloopRamp = kManip.CLIMB_VOLTAGE_RAMP;
    configA.openloopRamp = kManip.CLIMB_VOLTAGE_RAMP;
    configA.continuousCurrentLimit = kManip.CLIMB_CURRENT_LIMIT;
    configA.peakCurrentLimit = kManip.CLIMB_CURRENT_LIMIT;

    configB = configA;

    climbMotorA.configAllSettings(configA);
    climbMotorB.configAllSettings(configB);

    climbMotorA.setInverted(kManip.CLIMB_MOTOR_A_INVERTED);
    climbMotorA.setNeutralMode(kManip.CLIMB_MOTOR_NEUTRAL_MODE);
    climbMotorB.setInverted(kManip.CLIMB_MOTOR_B_INVERTED);
    climbMotorB.setNeutralMode(kManip.CLIMB_MOTOR_NEUTRAL_MODE);
  }
}
