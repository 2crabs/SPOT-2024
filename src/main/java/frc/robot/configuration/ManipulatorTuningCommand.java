// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kManip;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ManipulatorTuningCommand extends Command {

  private NetworkTable manipNetworkTable;
  private DoublePublisher NetTableAngleKP, NetTableAngleKI, NetTableAngleKD, NetTableAngleKSetPoint;

  String anglekPString = "Intake Angle PID: kP";
  String anglekIString = "Intake Angle PID: kI";
  String anglekDString = "Intake Angle PID: kD";
  String angleSetPointString = "Intake Angle Setpoint";

  double[] oldValues;

  ManipulatorSubsystem manipulatorSubsystem;

  /** Creates a new ShooterTuningCommand. */
  public ManipulatorTuningCommand(ManipulatorSubsystem manip) {
    manipNetworkTable = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    manipulatorSubsystem = manip;
    addRequirements(manip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber(anglekPString, kManip.INTAKE_ANGLE_PID_P);
    SmartDashboard.putNumber(anglekIString, kManip.INTAKE_ANGLE_PID_I);
    SmartDashboard.putNumber(anglekIString, kManip.INTAKE_ANGLE_PID_D);
    SmartDashboard.putNumber(angleSetPointString, kManip.INTAKE_ANGLE_TOGGLE_VALUES[0]);

    oldValues = new double[]{
      kManip.INTAKE_ANGLE_PID_P,
      kManip.INTAKE_ANGLE_PID_I,
      kManip.INTAKE_ANGLE_PID_D,
      kManip.INTAKE_ANGLE_TOGGLE_VALUES[0]
    };
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = SmartDashboard.getNumber(anglekPString, kManip.INTAKE_ANGLE_PID_P);
    double kI = SmartDashboard.getNumber(anglekIString, kManip.INTAKE_ANGLE_PID_I);
    double kD = SmartDashboard.getNumber(anglekDString, kManip.INTAKE_ANGLE_PID_D);
    double kSetPoint = SmartDashboard.getNumber(angleSetPointString, kManip.INTAKE_ANGLE_TOGGLE_VALUES[0]);

    if(areAnglePIDValuesChanged(kP, kI, kD, kSetPoint)) {
      publishAnglePIDValues(kP, kI, kD, kSetPoint);
      manipulatorSubsystem.configureAnglePIDValues(kP, kI, kD);
      manipulatorSubsystem.setIntakeMotorAngle(kSetPoint);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    manipulatorSubsystem.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  /** This Checks if the values are changed from their old values */
  public boolean areAnglePIDValuesChanged(double kP, double kI, double kD, double kSetPoint) {
    boolean changed = false;
    if(kP != oldValues[0]) {
      changed = true;
    }
    if(kI != oldValues[1]) {
      changed = true;
    }
    if(kD != oldValues[2]) {
      changed = true;
    }
    if(kSetPoint != oldValues[3]) {
      changed = true;
    }
    return changed;
  }

  /** This sets up publishing */
  private void SetUpNetworkTableEntries() {
    NetTableAngleKP = manipNetworkTable.getDoubleTopic(anglekPString).publish();
    NetTableAngleKI = manipNetworkTable.getDoubleTopic(anglekIString).publish();
    NetTableAngleKD = manipNetworkTable.getDoubleTopic(anglekDString).publish();
    NetTableAngleKSetPoint = manipNetworkTable.getDoubleTopic(angleSetPointString).publish();
  }

  private void publishAnglePIDValues(double kP, double kI, double kD, double kSetPoint) {
    NetTableAngleKP.set(kP);
    NetTableAngleKI.set(kI);
    NetTableAngleKD.set(kD);
    NetTableAngleKSetPoint.set(kSetPoint);
  }
}
