// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonCamera;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CameraConstants{
    public static final int resolutionX = 176;
    public static final int resolutionY = 144;
  }
  public static class ElevatorConstants {
    public static final int EXTENSION_ID = 10;
    public static final int EXTENSION_FOLLOW_ID = 11;
    
    public static final float EXTENSION_LIMIT_METERS = 0;
    
    public static final int ZERO_LIMIT_ID = 3;

    public static final double GROUND_POSITION = Units.inchesToMeters(1.5);
    public static final double SPEAKER_POSITION = 0;
    public static final double AMP_POSITION = Units.inchesToMeters(29);
    public static final double CHUTE_POSITION = Units.inchesToMeters(1.5);
    public static final double TRAP_POSITION = Units.inchesToMeters(30);
    public static final double START_POSITION = 0; 

    public static final double EXTENSION_P= 3.5;
    public static final double EXTENSION_I= 0;
    public static final double EXTENSION_D= 0;
    public static final double EXTENSION_TOLERANCE= 0.3;

    public static final double POSITION_CONVERSION_FACTOR = Units.inchesToMeters(30.)/63.5;
    public static final double VELOCITY_CONVERSION_FACTOR = 1;
  }
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class TestSingleModuleSwerveConstants {
    public static final double TURNGAP = 2;
  }

  public static class SwerveConstants { //ADD 90 degrees to all
    public static final int FL_DRIVE = 20; 
    public static final int FL_STEER = 1;
    public static final double FL_OFFSET = 3.36 + Math.PI * 5.0 / 4.;
    
    public static final int FR_DRIVE = 2; 
    public static final int FR_STEER = 3; 
    public static final double FR_OFFSET = 3.02 + Math.PI * 3.0 / 4;

    public static final int BL_DRIVE = 4; 
    public static final int BL_STEER = 5; 
    public static final double BL_OFFSET = 2.83 + Math.PI * 7.0 / 4;
    
    public static final int BR_DRIVE = 6; 
    public static final int BR_STEER = 7; 
    public static final double BR_OFFSET = 2.66 + Math.PI * 1.0 / 4.0;

    public static double MODULE_DIST = Units.inchesToMeters(27.25 / 2.0);
    public static final Translation2d FL_POS = new Translation2d(MODULE_DIST, MODULE_DIST);
    public static final Translation2d FR_POS = new Translation2d(MODULE_DIST, -MODULE_DIST);
    public static final Translation2d BL_POS = new Translation2d(-MODULE_DIST, MODULE_DIST);
    public static final Translation2d BR_POS = new Translation2d(-MODULE_DIST, -MODULE_DIST);

    public static final Translation2d BLUE_SPEAKER_POS = new Translation2d(Units.inchesToMeters(16.0035), Units.inchesToMeters(218.42));
    public static final double SPEAKER_TO_SPEAKER = Units.inchesToMeters(651.23);
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
    public static final double rollerscounterclockwise = 1; 
    public static final double sensorreached = .3;
    public static final double pivotclockwise = 1;
    public static final double pivotcounterclockwise = -1;
    public static final double pastsensortime = 3;
  }

  public static class ShooterConstants {
    public static final int LIMIT_SWITCH_ID = 1;
    public static final int PIVOT_MOTOR_ID = 12;

    public static final int SHOOTER_MOTOR_ONE_ID = 13;
    public static final int SHOOTER_MOTOR_TWO_ID = 14;

    public static final int FEEDER_MOTOR_ID = 15;

    public static final double FEED_ANGLE = Units.degreesToRadians(70);
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

  public static class AutoAlignConstants {
    
    public static final double robotRadius = 30/2;

    // X: APRILTAG_POS + SUBWOOFER_OFFSET + ROBOT_X_CENTER
    // ROT: rotated 180 deg from Apriltag pos since robots shoots from back
    public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(-1.50 + 37.711 + robotRadius), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0 + 180)); 
    
    // Source position closer to the centerline
    // Y: APRILTAG_POS 
    public static final Pose2d BLUE_SOURCE_POSE = new Pose2d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(120)); 
    
    public static final Pose2d BLUE_AMP_POSE = new Pose2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00 - robotRadius - 6.2), Rotation2d.fromDegrees(90));
    // public static final Pose2d BLUE_AMP_POSE = new Pose2d(Units.inchesToMeters(323.00 - robotRadius - 11.0), Units.inchesToMeters(72.5), Rotation2d.fromDegrees(90));
    public static final Pose2d ORIGIN_POSE = new Pose2d(0, 0, new Rotation2d(0));
    
    // X: APRILTAG_POS + SUBWOOFER_OFFSET + ROBOT_X_CENTER
    // ROT: rotated 180 deg from Apriltag pos since robots shoots from back
    public static final Pose2d RED_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(652.73 - 37.711 - robotRadius), Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180 - 180));
    
    // Source position closer to the centerline
    // Y: APRILTAG_POS 
    public static final Pose2d RED_SOURCE_POSE = new Pose2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Rotation2d.fromDegrees(60)); 
    
    public static final Pose2d RED_AMP_POSE = new Pose2d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00 - robotRadius), Rotation2d.fromDegrees(270));
  }

  public static class LEDConstants {
    public static final int LED_LENGTH = 140;
    public static final int LED_PWM_PORT = 0;
    public static final double BRIGHTNESS_SCALE_FACTOR = .5;
  }
  
  public static final class VisionConstants {
    public static final String VISION_TABLE_KEY = "Vision";

    public static final PhotonCamera FRONT_CAMERA = new PhotonCamera("Arducam_OV9281_USB_Camera_2");
    public static final Transform3d FRONT_CAMERA_POSE = new Transform3d(
      new Translation3d(Units.inchesToMeters(+8), Units.inchesToMeters(4), Units.inchesToMeters(+44)),
      new Rotation3d(Math.PI, Units.degreesToRadians(16), 0)
    );

    public static final PhotonCamera NOTE_CAMERA = new PhotonCamera("Microsoft_LifeCam_HD-3000");
    public static final Transform3d NOTE_CAMERA_POSE = new Transform3d();
  }
}
