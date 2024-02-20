// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightLEDMode;

/** For when limelight is not being used by anything else */
public class LimelightDefault extends CommandBase {

  Vision visionSubsystem;
  private boolean hasValidTarget;
  private boolean ledOn;
  private final Timer targetTimer = new Timer();
  private final Timer blinkerTimer = new Timer();

  /** Creates a new LimelightDefault. */
  public LimelightDefault(Vision vision) {
    visionSubsystem = vision;
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionSubsystem.setPipeline(kVision.DEFAULT_PIPELINE);
    targetTimer.start();
    targetTimer.reset();
    blinkerTimer.start();
    blinkerTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ledOn && visionSubsystem.hasValidTarget()) {
      targetTimer.reset();
      hasValidTarget = true;
    }
    else if(ledOn && targetTimer.hasElapsed(kVision.TARGET_VANISH_DELAY)) {
      hasValidTarget = false;
    }

    if(DriverStation.isEnabled()) {
      if(Math.round(blinkerTimer.get() % 2) == 1) {
        visionSubsystem.setLEDMode(LimelightLEDMode.ON);
      }
      else {
        visionSubsystem.setLEDMode(LimelightLEDMode.ON);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetTimer.stop();
    blinkerTimer.stop();
    visionSubsystem.setLEDMode(LimelightLEDMode.PIPELINE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
