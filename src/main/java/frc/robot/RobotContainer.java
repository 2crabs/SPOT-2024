// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kControls;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowCurrentTarget;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNoteIntoAmp;
import frc.robot.commands.ShootNoteIntoSpeaker;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private final Vision m_visionSubsystem = new Vision();
  private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem();
  private final SwerveDrive m_driveSubsystem = new SwerveDrive(m_visionSubsystem);

  private final CommandXboxController m_driverController =
      new CommandXboxController(kControls.DRIVE_CONTROLLER_ID);
  private final CommandXboxController m_manipulatorController =
      new CommandXboxController(kControls.MANIPULATOR_CONTROLLER_ID);

  public RobotContainer() {
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    m_driverController.a().whileTrue(new FollowCurrentTarget(
      m_visionSubsystem, 
      m_driveSubsystem, 
      () -> -Constants.kControls.X_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(Constants.kControls.TRANSLATION_Y_AXIS)),
      () -> -Constants.kControls.Y_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(Constants.kControls.TRANSLATION_X_AXIS))
      ));

    m_driveSubsystem.setDefaultCommand(new DriveCommand(
            () -> -kControls.Y_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(kControls.TRANSLATION_Y_AXIS)),
            () -> -kControls.X_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(kControls.TRANSLATION_X_AXIS)),
            () -> -kControls.THETA_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(kControls.ROTATION_AXIS)),
            m_driveSubsystem
    ));

    m_manipulatorSubsystem.setDefaultCommand(new StopIntake(m_manipulatorSubsystem));

    m_manipulatorController.leftTrigger().whileTrue(new IntakeNote(m_manipulatorSubsystem));

    m_manipulatorController.leftBumper().onTrue(new ShootNoteIntoSpeaker(m_manipulatorSubsystem));
    m_manipulatorController.rightBumper().onTrue(new ShootNoteIntoAmp(m_manipulatorSubsystem));
    
    //m_driveSubsystem.setDefaultCommand(m_driveSubsystem.jogTurnMotors(1 * Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND, false));

    //m_driveSubsystem.setDefaultCommand(m_driveSubsystem.CANCoderTuningCommand());
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

