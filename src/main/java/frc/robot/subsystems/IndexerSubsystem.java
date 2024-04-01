package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{

    // public CANSparkMax indexerMotor;
    public TalonFX indexerMotor;

    public DigitalInput beamBreakSensor = new DigitalInput(2);

    public double lastBeamBreakTriggerTimeStamp;

    public IndexerSubsystem() {
        lastBeamBreakTriggerTimeStamp = -999.999;
        indexerMotor = new TalonFX(18);
    }

    /**
     * This runs the indexer at a speed
     * @param speed the speed to run the indexer motor
     */
    public void runWithSpeed(double speed) {
        indexerMotor.set(speed);
    }

    /** Returns the current state of the beam break sensor */
    public boolean getIndexerBeamBreakSensor() {
        return beamBreakSensor.get();
    }

    /** Returns the last time the beam break sensor was triggered in seconds */
    public double getLastBeamBreakSensorTriggerTimeStamp() {
        return lastBeamBreakTriggerTimeStamp;
    }

    /** Returns the time since the last time the beam break sensor was toggled */
    public double timeSinceLastBeamBreakSensorToggle() {
        return Timer.getFPGATimestamp() - lastBeamBreakTriggerTimeStamp;
    }

    /** Returns true if the robot has a notein the indexer */
    public boolean hasNote() {
        return getIndexerBeamBreakSensor();
    }

    @Override
    public void periodic() {
        if(getIndexerBeamBreakSensor()) {
            lastBeamBreakTriggerTimeStamp = Timer.getFPGATimestamp();
        }
        SmartDashboard.putBoolean("Beam Break", getIndexerBeamBreakSensor());
        SmartDashboard.putNumber("TimeStamp Beam Break", lastBeamBreakTriggerTimeStamp);
    }

    /** This configures the motor controllers */
    public void configureHardware() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 5;

        indexerMotor.getConfigurator().apply(motorConfig);
    }
}