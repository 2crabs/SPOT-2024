package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase{

    public CANSparkMax motor;
    public CANSparkMax motor2;

    public PIDController pid1 = new PIDController(0.00025, 0.00005, 0);

    public IndexerSubsystem() {
        motor = new CANSparkMax(15, MotorType.kBrushless);
        motor2 = new CANSparkMax(16, MotorType.kBrushless);
        motor.setSmartCurrentLimit(50);
        motor2.setSmartCurrentLimit(50);
        motor2.setInverted(true);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public void runWithSpeed(double input) {
        //motor.set(pid1.calculate(motor.getEncoder().getVelocity(), input*2000.0));
        SmartDashboard.putNumber("encoder", motor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter PID", pid1.calculate(motor.getEncoder().getVelocity(), input*1000.0));
        motor.set(input);
        motor2.set(input);
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
        
    }
}