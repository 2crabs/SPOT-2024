// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kManip;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerJustShootAuto extends ParallelCommandGroup {
  private IntakeSubsystem m_intakeSubsystem;
  private IndexerSubsystem m_indexerSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private SwerveDrive m_swerveDrive;
  /** Creates a new SpeakerJustShootAuto. */
  public SpeakerJustShootAuto(IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, SwerveDrive drive) {
    m_intakeSubsystem = intake;
    m_indexerSubsystem = indexer;
    m_shooterSubsystem = shooter;
    m_swerveDrive = drive;
    addRequirements(intake, indexer, shooter, drive);
    addCommands(
      new RunCommand(() -> m_shooterSubsystem.setShooterState(kAuto.AUTO_SHOOT_STATE), m_shooterSubsystem)
      .withTimeout(1)
      .andThen(
        new ParallelCommandGroup(
          new RunCommand(() -> m_intakeSubsystem.setIntakeSpinSpeed(kManip.INTAKE_SPIN_SPEED), m_intakeSubsystem),
          new RunCommand(() -> m_indexerSubsystem.runWithSpeed(kManip.INDEXER_SPIN_SPEED), m_indexerSubsystem)
        )
        .withTimeout(1)
        .andThen(
          new ParallelCommandGroup(
            new RunCommand(() -> m_indexerSubsystem.runWithSpeed(0), m_indexerSubsystem),
            new RunCommand(() -> m_intakeSubsystem.setIntakeSpinSpeed(0), m_intakeSubsystem),
            new RunCommand(() -> m_shooterSubsystem.setShooterState(0), m_shooterSubsystem)
          )
        )
      )
    );
  }
}
