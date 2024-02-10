// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kManip;
import frc.robot.subsystems.ManipulatorSubsystem;

public class IntakeNote extends Command {
  private final ManipulatorSubsystem manipSubsystem;

  // CHANGE THIS LATER. this boolean should turn true when the note is finished intake. we don't know how to do that yet.
  private boolean intakeFinished;

  /** Creates a new IntakeNote. */
  public IntakeNote(ManipulatorSubsystem manipulator) {
    manipSubsystem = manipulator;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    manipSubsystem.setPredeterminedIntakeMotorAngle(1);
    intakeFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intakeFinished) {
      manipSubsystem.setPredeterminedIntakeMotorAngle(1);
    manipSubsystem.setIntakeSpinSpeed(kManip.INTAKE_SPIN_SPEED);
    } else {
      manipSubsystem.setPredeterminedIntakeMotorAngle(0);
      manipSubsystem.setIntakeSpinSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
