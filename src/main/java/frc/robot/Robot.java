// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.kDisplay;
import frc.robot.Constants.kVision;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  AddressableLED leds;
  AddressableLEDBuffer buff;

  int patternRainbowStart = 0;

  Thread m_visionThread;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    if(!kVision.PiVision) {
      m_visionThread = setupVisionThread();
      m_visionThread.setDaemon(true);
      m_visionThread.start();
    }

    leds = new AddressableLED(0);
    buff = new AddressableLEDBuffer(10);
    leds.setLength(buff.getLength());
    leds.setData(buff);
    leds.start();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    rainbowPattern();
    leds.setData(buff);
  }

  public void rainbowPattern() {
    for(int i = 0; i < buff.getLength(); i++) {
      int hue = (patternRainbowStart + (i*180 / buff.getLength())) % 180;
      buff.setHSV(i, hue, 255, 128);
    }

    patternRainbowStart += 3;

    patternRainbowStart %= 180;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public Thread indicatorThread() {
    return new Thread(
      ()-> {
        Mat mat = new Mat();
        mat.reshape(480, 480);

        CvSource outputSource = CameraServer.putVideo("Note Indicator", 480, 480);

        while(!Thread.interrupted()) {
          if(m_robotContainer != null) {
            if(m_robotContainer.getIndexerSubsystem().hasNote()) {
              for(int x = 0; x < mat.width(); x++) {
                for(int y = 0; y < mat.height(); y++) {
                  mat.put(x, y, new double[]{0.0, 255.0, 0.0});
                }
              }
            } else {
              for(int x = 0; x < mat.width(); x++) {
                for(int y = 0; y < mat.height(); y++) {
                  mat.put(x, y, new double[]{255.0, 0.0, 0.0});
                }
              }
            }
          }
          outputSource.putFrame(mat);
        }
      }
    );
  }

  public Thread setupVisionThread() {
    return new Thread(
      () -> {
        // Create a new ShapeDetection Object.
        // ShapeDetection shapeDetection = new ShapeDetection(999, kVision.MIN_CONTOUR_AREA);

        // Get the USBCamera from CameraServer
        UsbCamera camera = CameraServer.startAutomaticCapture(0);

        camera.setResolution(640, 480);

        VideoSink server = CameraServer.getServer();

        // This captures Mats from the camera
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Driver Camera", 640, 480);

        // Reuse this mat so the computer doesn't blow up
        Mat mat = new Mat();
        
        // Stop the thread when restarting robot code or deploying
        while (!Thread.interrupted()) {
          server.setSource(camera);

          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
          }
          // Put rectangle on the image (TEST)
          Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
          Imgproc.putText(
            mat,
            "" + cvSink.getSource().getActualFPS(),
            kDisplay.FPS_COUNTER_POSITION,
            0,
            kDisplay.FPS_COUNTER_SIZE,
            kDisplay.FPS_COUNTER_COLOR
          );

          /*
          if(kVision.detectNotes) {
            shapeDetection.detectShapesFromImage(mat);
            if(shapeDetection.containsNotes()) {
              Imgproc.putText(
                mat,
                shapeDetection.getNoteIndexes().size() + "Notes Detected",
                kDisplay.NOTE_COUNTER_POSITION,
                0,
                kDisplay.NOTE_COUNTER_SIZE,
                kDisplay.NOTE_COUNTER_COLOR
              );
            }

            mat = shapeDetection.renderShapeDetails(mat, kDisplay.LINE_COLOR, kDisplay.DETAIL_COLOR, true, true, true, true, true);
          }
          */

          outputStream.putFrame(mat);
        }
      }
    );
  }
}
