package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{

    // public CANSparkMax indexerMotor;
    public TalonFX indexerMotor;

    public boolean indexerBeamBreakSensorInput;

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
        return indexerBeamBreakSensorInput;
    }

    /** Returns the last time the beam break sensor was triggered in seconds */
    public double getLastBeamBreakSensorTriggerTimeStamp() {
        return lastBeamBreakTriggerTimeStamp;
    }

    /** Returns the time since the last time the beam break sensor was toggled */
    public double timeSinceLastBeamBreakSensorToggle() {
        return Timer.getFPGATimestamp() - lastBeamBreakTriggerTimeStamp;
    }

    @Override
    public void periodic() {
        if(getIndexerBeamBreakSensor()) {
            lastBeamBreakTriggerTimeStamp = Timer.getFPGATimestamp();
        }
    }

    /** This configures the motor controllers */
    public void configureHardware() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 5;

        indexerMotor.getConfigurator().apply(motorConfig);
    }
}