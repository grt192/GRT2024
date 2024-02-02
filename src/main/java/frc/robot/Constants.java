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
  public static class ElevatorConstants {
    public static final int EXTENSION_ID = 10;
    public static final int EXTENSION_FOLLOW_ID = 11;
    
    public static final float EXTENSION_LIMIT_METERS = 0;
    
    public static final int ZERO_LIMIT_ID = 2;

    public static final double GROUND_POSITION = 0;
    public static final double SPEAKER_POSITION = 0;
    public static final double AMP_POSITION = Units.inchesToMeters(28);
    public static final double CHUTE_POSITION = Units.inchesToMeters(25);
    public static final double TRAP_POSITION = Units.inchesToMeters(30);
    public static final double START_POSITION = 0; 

    public static final double EXTENSION_P= 3.5;
    public static final double EXTENSION_I= 0;
    public static final double EXTENSION_D= 0;
    public static final double EXTENSION_TOLERANCE= 0.003;

    public static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(30.)/63.5;
    public static final double VELOCITY_CONVERSION_FACTOR = 1;
  }
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

  public static class IntakeConstants {
    public static final int INTEGRATION_MOTOR_ID = 19;
    public static final int FRONT_MOTOR_ID = 17;
    public static final int BACK_MOTOR_ID = 18;
    public static final int PIVOT_MOTOR_ID = 16;

    public static final int sensorID = 0;
    public static final int extendedlimitswitchID = 5;
    public static final int retractedlimitswitchID = 6;
    //public static final int intakeencoderID = 3;

    public static final double rollersclockwise = 1;
    public static final double rollerscounterclockwise = -1; 
    public static final double sensorreached = 1;
    public static final double pivotclockwise = 1;
    public static final double pivotcounterclockwise = -1;
    public static final double pastsensortime = 0.5;
  }




  public static class ClimbConstants {
    public static final int LEFT_WINCH_MOTOR_ID = 8;
    public static final int LEFT_ZERO_LIMIT_ID = 0;

    public static final int RIGHT_WINCH_MOTOR_ID = 9;
    public static final int RIGHT_ZERO_LIMIT_ID = 1;
   
    public static final double WINCH_REDUCTION = 9.49;
    public static final double AXLE_PERIMETER_METERS = 6 * Units.inchesToMeters(.289) ;
    
    public static final double EXTENSION_LIMIT_METERS = Units.inchesToMeters(39);
    public static final double EXTENSION_TOLERANCE_METERS = 0.01;

    public static final double MAX_WINCH_POWER = 0.6;
  }
}
