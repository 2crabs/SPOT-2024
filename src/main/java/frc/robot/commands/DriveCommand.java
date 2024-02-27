package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;


public class DriveCommand extends Command {
    private final SwerveDrive swerveDrive;

    private DoubleSupplier forwardAxis;
    private DoubleSupplier sidewaysAxis;
    private DoubleSupplier rotationAxis;

    public DriveCommand(DoubleSupplier forwardAxis, DoubleSupplier sidewaysAxis, DoubleSupplier rotationAxis, SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        this.forwardAxis = forwardAxis;
        this.sidewaysAxis = sidewaysAxis;
        this.rotationAxis = rotationAxis;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.swerveDrive);
    }

    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {

        double deadzoneRotation = deadzone(rotationAxis.getAsDouble(), Constants.kControls.ROTATION_DEADZONE) * 3;
        double deadzoneForward = deadzone(forwardAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE) * 2;
        double deadzoneSideways = deadzone(sidewaysAxis.getAsDouble(), Constants.kControls.TRANSLATION_DEADZONE) * 2;

        //only use the pid rotation if going at a certain speed
        // if (deadzoneRotation == 0.0 && Math.sqrt((deadzoneForward*deadzoneForward)+(deadzoneSideways*deadzoneSideways)) > 0.15) {
        //     swerveDrive.drive(deadzoneForward, deadzoneSideways, swerveDrive.targetRotation, true, true);
        // } else {
            swerveDrive.basicDrive(deadzoneForward, deadzoneSideways, deadzoneRotation, true);
        // }
    }

    public double deadzone(double input, double tolerance){
        if (Math.abs(input) < tolerance){
            return 0.0;
        }
        return input;
    }
    /**
     * <p>
     * Returns whether this command has finished. Once a command finishes -- indicated by
     * this method returning true -- the scheduler will call its {@link #end(boolean)} method.
     * </p><p>
     * Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always
     * return true will result in the command executing once and finishing immediately. It is
     * recommended to use * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand}
     * for such an operation.
     * </p>
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    /**
     * The action to take when the command ends. Called when either the command
     * finishes normally -- that is it is called when {@link #isFinished()} returns
     * true -- or when  it is interrupted/canceled. This is where you may want to
     * wrap up loose ends, like shutting off a motor that was being used in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {

    }
}
