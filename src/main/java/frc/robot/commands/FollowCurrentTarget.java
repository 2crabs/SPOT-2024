// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class FollowCurrentTarget extends Command {
  private final Vision m_visionSubsystem;
  private final SwerveDrive m_driveSubsystem;

  private double m_turn = 0;

  private double m_yAxis, m_xAxis;

  private PIDController m_angleController = new PIDController(0.5, 0, 0.015);

  /** Creates a new FollowCurrentTarget. */
  public FollowCurrentTarget(Vision visionSubsystem, SwerveDrive driveSubsystem, DoubleSupplier forwardBackAxis, DoubleSupplier leftRightAxis) {
    m_visionSubsystem = visionSubsystem;
    m_driveSubsystem = driveSubsystem;

    m_yAxis = forwardBackAxis.getAsDouble();
    m_xAxis = leftRightAxis.getAsDouble();

    addRequirements(visionSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_angleController.setSetpoint(0);
    m_angleController.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("FollowingApriltag", true);

    //double turnA = Math.abs(m_visionSubsystem.getTargetOffsetX()/54);
    double offset = m_driveSubsystem.getGyroRotation().getRotations()+(m_visionSubsystem.getTargetOffsetX()/360);
    if(m_visionSubsystem.hasValidTarget()) {
      m_turn = offset;
    } else {
      m_turn = m_driveSubsystem.getGyroRotation().getRotations();
    }

    // m_driveSubsystem.basicDrive(m_yAxis, m_xAxis, m_turn, true); 
    m_driveSubsystem.drive(m_yAxis, m_xAxis, m_turn, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("FollowingApriltag", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
