// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kManip;

public class ClimbSubsystem extends SubsystemBase {

  private TalonSRX climbMotorA = new TalonSRX(kManip.CLIMB_MOTOR_A_ID);
  private TalonSRX climbMotorB = new TalonSRX(kManip.CLIMB_MOTOR_B_ID);

  private PIDController climbMotorPID = new PIDController(
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
    //double pidCalculation = climbMotorPID.calculate(targetPosition);
    //climbMotorA.set()
  }

  public void setClimbSpeed(double speed) {
    climbMotorA.set(ControlMode.PercentOutput, speed);
    climbMotorB.set(ControlMode.PercentOutput, speed);
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
