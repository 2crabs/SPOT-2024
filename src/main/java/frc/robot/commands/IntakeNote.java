// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kManip;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends Command {
  private final IntakeSubsystem manipSubsystem;
  private final IndexerSubsystem indexerSubsystem;

  private boolean beamBroken;

  // CHANGE THIS LATER. this boolean should turn true when the note is finished intake. we don't know how to do that yet.
  private boolean intakeFinished;

  /** Creates a new IntakeNote. */
  public IntakeNote(IntakeSubsystem manipulator, IndexerSubsystem indexer) {
    manipSubsystem = manipulator;
    indexerSubsystem  = indexer;
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //manipSubsystem.setPredeterminedIntakeMotorAngle(1);
    intakeFinished = false;
    beamBroken = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexerSubsystem.getIndexerBeamBreakSensor()) {
      beamBroken = true;
    }

    // manipSubsystem.setPredeterminedIntakeMotorAngle(1);
    manipSubsystem.setIntakeSpinSpeed(kManip.INTAKE_SPIN_SPEED);
    indexerSubsystem.runWithSpeed(kManip.INDEXER_SPIN_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(beamBroken && indexerSubsystem.timeSinceLastBeamBreakSensorToggle() >= kManip.BEAM_BREAK_SENSOR_INDEXER_DELAY) {
      return true;
    }
    return false;
  }
}
