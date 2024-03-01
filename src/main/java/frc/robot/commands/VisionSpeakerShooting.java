// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kField;
import frc.robot.Constants.kManip;
import frc.robot.Constants.kVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.PoseSpace;
import frc.robot.utils.math.MathUtils;

public class VisionSpeakerShooting extends Command {
  private Vision visionSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private SwerveDrive driveSubsystem;

  private boolean driveTowardsTarget = false;

  /** Creates a new VisionSpeakerShooting. */
  public VisionSpeakerShooting(Vision vision, ShooterSubsystem shooter, SwerveDrive drive, boolean doDriving) {
    visionSubsystem = vision;
    shooterSubsystem = shooter;
    driveTowardsTarget = doDriving;
    addRequirements(vision, shooter);
    if(doDriving) {
      addRequirements(drive);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d robotPose = new Pose2d(new Translation2d(25492549, 25492549), new Rotation2d(0.0));

    if(visionSubsystem.hasValidTarget()) {
      robotPose = visionSubsystem.getPoseCustomSpace(PoseSpace.CAMERA, PoseSpace.TARGET);
    } else {
      if(kVision.USE_FIELD_POS_IN_SNAP_COMMANDS) {
        Pose2d tempPose = driveSubsystem.getPose();
        if(DriverStation.getAlliance().get() == Alliance.Blue) {
          robotPose = new Pose2d(
            new Translation2d(
              kField.BLUE_SPEAKER_X - tempPose.getX(),
              kField.BLUE_SPEAKER_Z - tempPose.getY()
            ),
            tempPose.getRotation()
          );
        } else {
          robotPose = new Pose2d(
            new Translation2d(
              kField.RED_SPEAKER_X - tempPose.getX(),
              kField.RED_SPEAKER_Z - tempPose.getY()
            ),
            tempPose.getRotation()
          );
        }
      } else {
        
      }
    }

    double distance = MathUtils.distance(0, robotPose.getX(), 0, robotPose.getY(), 0, 0);

    if(distance < kManip.MAX_SHOOTING_DISTANCE_SPEAKER) {
      shooterSubsystem.setShooterDistance(distance, kManip.VISION_SHOOTING_LINEAR_MUL, kManip.VISION_SHOOTING_EXPONENT, kManip.VISION_SHOOTING_OFFSET);
    } else {
      if(driveTowardsTarget && visionSubsystem.hasValidTarget()) {
        driveTowardsSpeaker(robotPose);
      }
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

  private void driveTowardsSpeaker(Pose2d botPose) {
    driveSubsystem.basicDrive(0.1, 0.0, 0.0, false);
  }
}
