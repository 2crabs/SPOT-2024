// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class SnapToNote extends Command {

  private SwerveDrive m_driveSubsystem;
  private Vision m_visionSubsystem;

  private DoubleSupplier forwardAxis, sidewaysAxis;

  /** Creates a new SnapToNote. */
  public SnapToNote(DoubleSupplier forward_axis, DoubleSupplier sideways_axis, SwerveDrive drive, Vision vision) {
    m_driveSubsystem = drive;
    m_visionSubsystem = vision;
    forwardAxis = forward_axis;
    sidewaysAxis = sideways_axis;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deadzoneForward = deadzone(forwardAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE) * 3.5;
    double deadzoneSideways = deadzone(sidewaysAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE) * 3.5;

    double rotation = 0;
    if(m_visionSubsystem.hasValidNote()) {
      rotation = m_visionSubsystem.getNoteScreenOffsetX() * 0.1;
    }

    m_driveSubsystem.basicDrive(deadzoneForward, deadzoneSideways, rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double deadzone(double input, double tolerance){
    if (Math.abs(input) < tolerance){
        return 0.0;
    }
    return input;
  }
}
