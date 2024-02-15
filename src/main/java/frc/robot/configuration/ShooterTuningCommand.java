// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.configuration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kManip;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ShooterTuningCommand extends Command {

  String anglekPString = "Intake Angle PID: kP";
  String anglekIString = "Intake Angle PID: kI";
  String anglekDString = "Intake Angle PID: kD";

  IndexerSubsystem indexerSubsystem;
  ManipulatorSubsystem manipulatorSubsystem;

  /** Creates a new ShooterTuningCommand. */
  public ShooterTuningCommand(IndexerSubsystem indexer, ManipulatorSubsystem manip) {
    indexerSubsystem = indexer;
    manipulatorSubsystem = manip;
    addRequirements(indexer, manip);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber(anglekPString, kManip.INTAKE_ANGLE_PID_P);
    SmartDashboard.putNumber(anglekIString, kManip.INTAKE_ANGLE_PID_I);
    SmartDashboard.putNumber(anglekIString, kManip.INTAKE_ANGLE_PID_D);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kP = SmartDashboard.getNumber(anglekPString, kManip.INTAKE_ANGLE_PID_P);
    double kI = SmartDashboard.getNumber(anglekIString, kManip.INTAKE_ANGLE_PID_I);
    double kD = SmartDashboard.getNumber(anglekDString, kManip.INTAKE_ANGLE_PID_D);

    manipulatorSubsystem.configureAnglePIDValues(kP, kI, kD);
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
}
