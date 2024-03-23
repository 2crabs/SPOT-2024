// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kControls;
import frc.robot.Constants.kManip;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowCurrentTarget;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.OuttakeNote;
import frc.robot.commands.StopIndexer;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopShooter;
import frc.robot.commands.VisionSpeakerShooting;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class RobotContainer {
  public HashMap<String, Command> autoMap = new HashMap<>();

  private final SendableChooser<Command> autoChooser;
  private final Vision m_visionSubsystem = new Vision();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final SwerveDrive m_driveSubsystem = new SwerveDrive(m_visionSubsystem);

  public final CommandXboxController m_driverController =
      new CommandXboxController(kControls.DRIVE_CONTROLLER_ID);
  private final CommandXboxController m_manipulatorController =
      new CommandXboxController(kControls.MANIPULATOR_CONTROLLER_ID);

  public RobotContainer() {
    configureAutoMap();
    configureBindings();
    autoChooser = new SendableChooser<Command>();
    configureAutoChooser();

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
    
    m_driverController.rightBumper().whileTrue(new FollowCurrentTarget(
      m_visionSubsystem, 
      m_driveSubsystem, 
      () -> -kControls.X_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(kControls.TRANSLATION_Y_AXIS)),
      () -> -kControls.Y_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(kControls.TRANSLATION_X_AXIS))
    ));

    // m_driverController.rightBumper().whileTrue(new VisionSpeakerShooting(m_visionSubsystem, m_driveSubsystem, true, false));

    m_driveSubsystem.setDefaultCommand(new DriveCommand(
      () -> -kControls.Y_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(kControls.TRANSLATION_Y_AXIS)),
      () -> -kControls.X_DRIVE_LIMITER.calculate(m_driverController.getRawAxis(kControls.TRANSLATION_X_AXIS)),
      () -> -kControls.THETA_DRIVE_LIMITER.calculate(-m_driverController.getRawAxis(kControls.ROTATION_AXIS)),
      m_driveSubsystem
    ));

    m_driverController.y().whileTrue(new RunCommand(() -> m_driveSubsystem.zeroGyroscope(), m_driveSubsystem));

    m_intakeSubsystem.setDefaultCommand(new StopIntake(m_intakeSubsystem));
    m_indexerSubsystem.setDefaultCommand(new StopIndexer(m_indexerSubsystem));

    if(kControls.USE_LEFT_Y_FOR_INTAKING) {
      m_intakeSubsystem.setDefaultCommand(
        new RunCommand(() -> m_intakeSubsystem.setIntakeSpinSpeed(kManip.INTAKE_SPIN_SPEED * (
          Math.abs(m_manipulatorController.getLeftY()) > kManip.INTAKE_DEADZONE ? 
          m_manipulatorController.getLeftY() : 
          0.0
        )
      ), m_intakeSubsystem));
      m_indexerSubsystem.setDefaultCommand(
        new RunCommand(() -> m_indexerSubsystem.runWithSpeed(kManip.INDEXER_SPIN_SPEED * (
          Math.abs(m_manipulatorController.getLeftY()) > kManip.INTAKE_DEADZONE ? 
          m_manipulatorController.getLeftY() * kManip.INTAKE_SPIN_SPEED : 
          0.0
        )
      ), m_indexerSubsystem));
    } else {
      m_manipulatorController.leftBumper().whileTrue(new IntakeNote(m_intakeSubsystem, m_indexerSubsystem));
      m_manipulatorController.rightBumper().whileTrue(new OuttakeNote(m_intakeSubsystem, m_indexerSubsystem));
    }
    
    m_shooterSubsystem.setDefaultCommand(
      new RunCommand(() -> m_shooterSubsystem.setShooterSpeed(
        Math.abs(m_manipulatorController.getLeftY()) > 0.1 ? 
        -m_manipulatorController.getLeftY() * 1 : 
        0.0
      ), m_shooterSubsystem)
    );

    // m_manipulatorController.b().whileTrue(new MoveIntake(m_intakeSubsystem, () -> (m_manipulatorController.getRightY()/6)));

    //m_manipulatorController.a().onTrue(new ShootNoteIntoSpeaker(m_shooterSubsystem, m_indexerSubsystem));
    //m_manipulatorController.x().onTrue(new ShootNoteIntoAmp(m_shooterSubsystem, m_indexerSubsystem));

    m_manipulatorController.a().onTrue(new RunCommand(() -> m_shooterSubsystem.setShooterState(2), m_shooterSubsystem));
    m_manipulatorController.x().onTrue(new RunCommand(() -> m_shooterSubsystem.setShooterState(1), m_shooterSubsystem));

    m_manipulatorController.a().onFalse(
      new RunCommand(() -> m_shooterSubsystem.setShooterSpeed(
        Math.abs(m_manipulatorController.getLeftY()) > 0.1 ? 
        -m_manipulatorController.getLeftY() * 1 : 
        0.0
      ), m_shooterSubsystem)
    );
    m_manipulatorController.x().onFalse(
      new RunCommand(() -> m_shooterSubsystem.setShooterSpeed(
        Math.abs(m_manipulatorController.getLeftY()) > 0.1 ? 
        -m_manipulatorController.getLeftY() * 1 : 
        0.0
      ), m_shooterSubsystem)
    );

    // m_manipulatorController.y().whileTrue(new RunCommand(() -> m_shooterSubsystem.setShooterState(2), m_shooterSubsystem));
    
    //m_driveSubsystem.setDefaultCommand(m_driveSubsystem.jogTurnMotors(1 * Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND, false));

    //m_driveSubsystem.setDefaultCommand(m_driveSubsystem.CANCoderTuningCommand());
  }

  public void configureAutoMap() {
    autoMap.put("startIntake", new IntakeNote(m_intakeSubsystem, m_indexerSubsystem));
    autoMap.put("stopIntake", new ParallelCommandGroup(new StopIntake(m_intakeSubsystem), new StopIndexer(m_indexerSubsystem)));
    autoMap.put("speakerShoot", new RunCommand(() -> m_shooterSubsystem.setShooterState(2), m_shooterSubsystem));
    autoMap.put("ampShoot", new RunCommand(() -> m_shooterSubsystem.setShooterState(1), m_shooterSubsystem));
    autoMap.put("stopShooter", new StopShooter(m_shooterSubsystem));
    autoMap.put("smartIntake", new IntakeNote(m_intakeSubsystem, m_indexerSubsystem));
    autoMap.put("smartShoot", new RunCommand(() -> m_shooterSubsystem.setShooterState(2), m_shooterSubsystem).withTimeout(0.2));
    NamedCommands.registerCommands(autoMap);
  }

  public void configureAutoChooser() {
    autoChooser.addOption("Burn in the depths of hell", Commands.none());
    autoChooser.addOption("Center Notes - Source Start", AutoBuilder.buildAuto("Center Notes - Amp Start"));
    autoChooser.addOption("Center Notes - Amp Start", AutoBuilder.buildAuto("Center Notes - Source Start"));
    //autoChooser.addOption("1 Note - Speaker Start", getAutonomousCommand());
    autoChooser.addOption("2 Note - Speaker Start", AutoBuilder.buildAuto("Speaker Start - 2 Note"));
    autoChooser.addOption("3 Note - Speaker Start", AutoBuilder.buildAuto("3 Notes"));
    autoChooser.addOption("4 Note - Speaker Start", AutoBuilder.buildAuto("Speaker 4 Note"));
    autoChooser.addOption("5 Note - Speaker Start", AutoBuilder.buildAuto("Speaker Start - 5 Note"));
    autoChooser.addOption("Swiper, No Swiping! - Amp Start", AutoBuilder.buildAuto("Swiper Amp"));
    autoChooser.addOption("Swiper, No Swiping! - Source Start", AutoBuilder.buildAuto("Swiper Source"));
    autoChooser.addOption("Testing Auto", AutoBuilder.buildAuto("Testing"));
  }

  public Command getAutonomousCommand() {
    //return AutoBuilder.buildAuto("Test1");
    return autoChooser.getSelected();
  }
}

