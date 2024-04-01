// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.kField;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.utils.math.MathUtils;
import frc.robot.utils.pose.FieldLayout;

public class SnapToSpeaker extends Command {

  SwerveDrive driveSubsystem;
  Vision visionSubsystem;

  DoubleSupplier forwardAxis;
  DoubleSupplier sidewaysAxis;

  /** Creates a new SnapToSpeaker. */
  public SnapToSpeaker(SwerveDrive drive, Vision vision, DoubleSupplier forward, DoubleSupplier sideways) {
    driveSubsystem = drive;
    visionSubsystem = vision;

    this.forwardAxis = forward;
    this.sidewaysAxis = sideways;
    addRequirements(drive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    double speakerOffset = 0.0;

    if(kVision.USE_FIELD_POS_IN_SNAP_COMMANDS) { 
      if(FieldLayout.getClosestSpeaker(driveSubsystem.getPose().getX(), driveSubsystem.getPose().getY()) == FieldLayout.Landmark.BLUE_SPEAKER) {
        if(visionSubsystem.hasValidTarget() && visionSubsystem.getTargetID() == kVision.SPEAKER_APRILTAG_ID_BLUE) {
          speakerOffset = visionSubsystem.getTargetOffsetX();
        } else {
          speakerOffset = driveSubsystem.getGyroRotation().getDegrees() 
          - MathUtils.angleOfLine(
            kField.RED_SPEAKER_X, kField.RED_SPEAKER_Z, 
            driveSubsystem.getPose().getX(), 
            driveSubsystem.getPose().getY()
          );
        }
      } else {
        if(visionSubsystem.hasValidTarget() && visionSubsystem.getTargetID() == kVision.SPEAKER_APRILTAG_ID_RED) {
          speakerOffset = visionSubsystem.getTargetOffsetX();
        } else {
          speakerOffset = driveSubsystem.getGyroRotation().getDegrees() 
          - MathUtils.angleOfLine(
            kField.RED_SPEAKER_X, kField.RED_SPEAKER_Z, 
            driveSubsystem.getPose().getX(), 
            driveSubsystem.getPose().getY()
          );
        }
      }
    } else {
      if(visionSubsystem.hasValidTarget() && (
        visionSubsystem.getTargetID() == kVision.SPEAKER_APRILTAG_ID_RED || 
        visionSubsystem.getTargetID() == kVision.SPEAKER_APRILTAG_ID_BLUE
      )) {
        speakerOffset = visionSubsystem.getTargetOffsetX();
      } else {
        speakerOffset = 1.0;
      }
    }

    double deadzoneForward = deadzone(forwardAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE);
    double deadzoneSideways = deadzone(sidewaysAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE);
    double finalRotation = driveSubsystem.getGyroRotation().getDegrees() + speakerOffset;

    driveSubsystem.drive(deadzoneForward, deadzoneSideways, finalRotation, true, true);
    */

    double rotation = 0;
    if(visionSubsystem.hasValidTarget()) {
      rotation = visionSubsystem.getTargetOffsetX()/(360 * 0.7);
    } else {
      rotation = 1.2;
      if(driveSubsystem.getGyroRotation().getDegrees() > 0) {
        rotation = -1.2;
      }
    }

    double deadzoneForward = deadzone(forwardAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE) * 3.5;
    double deadzoneSideways = deadzone(sidewaysAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE) * 3.5;

    driveSubsystem.basicDrive(deadzoneForward, deadzoneSideways, rotation, true);
  }

  public double deadzone(double input, double tolerance){
        if (Math.abs(input) < tolerance){
            return 0.0;
        }
        return input;
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
