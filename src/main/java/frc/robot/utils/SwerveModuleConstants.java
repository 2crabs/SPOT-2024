// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/**
 * Holds data specific to each Swerve module such as CAN ids
 */
public class SwerveModuleConstants {
    public final int wheelMotorID;
    public final int turnMotorID;
    public final int canCoderID;
    public final double canCoderOffsetDegrees;

    public final ModulePosition position;

    /**
     *
     * @param wheelMotorID CAN id of the Sparkmax for the wheel
     * @param turnMotorID CAN id of the Sparkmax for the angle of the module
     * @param canCoderID CAN id of the CANCoder
     * @param canCoderOffsetDegrees How far the CANCoder is off when the wheels are straight
     * @param position Position of the module. See {@link ModulePosition} for possible values
     */
    public SwerveModuleConstants(int wheelMotorID, int turnMotorID, int canCoderID, double canCoderOffsetDegrees, ModulePosition position) {
        this.wheelMotorID = wheelMotorID;
        this.turnMotorID = turnMotorID;
        this.canCoderID = canCoderID;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
        this.position = position;
    }
}
