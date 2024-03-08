package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{

    // public CANSparkMax indexerMotor;
    public TalonFX indexerMotor;
    public IndexerSubsystem() {
        indexerMotor = new TalonFX(18);
    }

    /**
     * This runs the indexer at a speed
     * @param speed the speed to run the indexer motor
     */
    public void runWithSpeed(double speed) {
        indexerMotor.set(speed);
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /** This configures the motor controllers */
    public void configureHardware() {
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 5;

        indexerMotor.getConfigurator().apply(motorConfig);
    }
}