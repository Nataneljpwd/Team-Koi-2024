// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Swerve {
    public static double MAX_VEL_FPS = 18.6;
  }

  public static final class AutoConsts {
    public static final Pose2d MIDDLE_RING_POS = new Pose2d(2.6, 5.55, new Rotation2d(-20));
    public static final Pose2d RED_SPEAKER = new Pose2d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42),
        new Rotation2d(180));
    public static final Pose2d BLUE_SPEAKER = new Pose2d(Units.inchesToMeters(-1.5), Units.inchesToMeters(218.42),
        new Rotation2d(0));
  }
}
