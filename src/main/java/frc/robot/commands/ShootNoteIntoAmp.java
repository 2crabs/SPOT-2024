// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ManipulatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteIntoAmp extends SequentialCommandGroup {
  private final ManipulatorSubsystem manipSubsystem;
  /** Creates a new ShootNoteIntoAmp. */
  public ShootNoteIntoAmp(ManipulatorSubsystem subsystem) {
    manipSubsystem = subsystem;
    addRequirements(subsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(() -> manipSubsystem.setShooterState(2), manipSubsystem),
      new WaitCommand(1),
      new RunCommand(() -> manipSubsystem.setShooterState(0), manipSubsystem)
    );
  }
}
