// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.pose;

import frc.robot.Constants.kField;
import frc.robot.utils.math.MathUtils;

/** Add your docs here. */
public class FieldLayout {

    public static Landmark getClosestSpeaker(double x, double z) {
        double distToRed = MathUtils.distance(x, kField.RED_SPEAKER_X, 0, 0, z, kField.RED_SPEAKER_Z);
        double distToBlue = MathUtils.distance(x, kField.BLUE_SPEAKER_X, 0, 0, z, kField.BLUE_SPEAKER_Z);
        if(distToRed < distToBlue) {
            return Landmark.RED_SPEAKER;
        } else {
            return Landmark.BLUE_SPEAKER;
        }
    }

    public static Landmark getClosestAmp(double x, double z) {
        double distToRed = MathUtils.distance(x, kField.RED_AMP_X, 0, 0, z, kField.RED_AMP_Z);
        double distToBlue = MathUtils.distance(x, kField.BLUE_AMP_X, 0, 0, z, kField.BLUE_AMP_Z);
        if(distToRed < distToBlue) {
            return Landmark.RED_SPEAKER;
        } else {
            return Landmark.BLUE_SPEAKER;
        }
    }

    public static enum Landmark {
        RED_SPEAKER,
        BLUE_SPEAKER,
        RED_AMP,
        BLUE_AMP,
        RED_SOURCE,
        BLUE_SOURCE
    }
}
