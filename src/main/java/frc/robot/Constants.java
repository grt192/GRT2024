// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TestSingleModuleSwerveConstants {
    public static final double TURNGAP = 2;
  }

  public static class SwerveConstants {
    public static final int FL_DRIVE = 0; 
    public static final int FL_STEER = 1;
    public static final double FL_OFFSET = 3.34 - Math.PI / 4;
    
    public static final int FR_DRIVE = 2; 
    public static final int FR_STEER = 3; 
    public static final double FR_OFFSET = 2.94 - Math.PI * 3 / 4;

    public static final int BL_DRIVE = 4; 
    public static final int BL_STEER = 5; 
    public static final double BL_OFFSET = 3.14  + Math.PI / 4;
    
    public static final int BR_DRIVE = 6; 
    public static final int BR_STEER = 7; 
    public static final double BR_OFFSET = 2.43 + Math.PI * 3.0 / 4.0;

    private static double MODULE_DIST = Units.inchesToMeters(27.25 / 2.0);
    public static final Translation2d FL_POS = new Translation2d(-MODULE_DIST, MODULE_DIST);
    public static final Translation2d FR_POS = new Translation2d(MODULE_DIST, MODULE_DIST);
    public static final Translation2d BL_POS = new Translation2d(-MODULE_DIST, -MODULE_DIST);
    public static final Translation2d BR_POS = new Translation2d(MODULE_DIST, -MODULE_DIST);
  }
}
