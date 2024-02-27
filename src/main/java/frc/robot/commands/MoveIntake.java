// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class MoveIntake extends Command {
  private IntakeSubsystem intakeSubsystem;
  private DoubleSupplier speedAxis;
  /** Creates a new lowerIntake. */
  public MoveIntake(IntakeSubsystem intake, DoubleSupplier speed) {
    intakeSubsystem = intake;
    speedAxis = speed;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intakeSubsystem.setPredeterminedIntakeMotorAngle(0);
    intakeSubsystem.setIntakeMotorAngleSpeed(speedAxis.getAsDouble());
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
