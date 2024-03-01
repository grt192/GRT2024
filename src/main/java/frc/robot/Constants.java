// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import org.photonvision.PhotonCamera;

/** Store the constants for the robot. */
public final class Constants {

    /** Constants for the cameras. */
    public static class CameraConstants {
        public static final int resolutionX = 176;
        public static final int resolutionY = 144;
    }

    /** Constants for the elevator subsystem. */
    public static class ElevatorConstants {
        public static final int EXTENSION_ID = 10;
        public static final int EXTENSION_FOLLOW_ID = 11;

        public static final float EXTENSION_LIMIT_METERS = 0;

        public static final int ZERO_LIMIT_ID = 7;

        public static final double ZERO_POSITION = 0;
        public static final double INTAKE_POSITION = .05;
        public static final double AMP_POSITION = .9;
        public static final double TRAP_POSITION = 1.;

        public static final double EXTENSION_P = 1.5;
        public static final double EXTENSION_I = 0;
        public static final double EXTENSION_D = 4;
        public static final double EXTENSION_TOLERANCE = 0.3;

        public static final double POSITION_CONVERSION_FACTOR = 1 / 27.785; // Units.inchesToMeters(30.)/63.5;
        public static final double VELOCITY_CONVERSION_FACTOR = 1;
    }

    /** Constants for the driver station. */
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    /** Constants for the test single module swerve subsystem. */
    public static class TestSingleModuleSwerveConstants {
        public static final double TURN_GAP = 2;
    }

    /** Constants for the swerve subsystem. */
    public static class SwerveConstants {
        public static boolean IS_RED = DriverStation.getAlliance().get() == DriverStation.Alliance.Red;

        public static final int FL_DRIVE = 20;
        public static final int FL_STEER = 1;
        public static final double FL_OFFSET = .8606 + Math.PI * 5.0 / 4.;

        public static final int FR_DRIVE = 2;
        public static final int FR_STEER = 3;
        public static final double FR_OFFSET = 3.068 + Math.PI * 3.0 / 4;

        public static final int BL_DRIVE = 4;
        public static final int BL_STEER = 5;
        public static final double BL_OFFSET = 2.816 + Math.PI * 7.0 / 4;

        public static final int BR_DRIVE = 6;
        public static final int BR_STEER = 7;
        public static final double BR_OFFSET = 2.44 + Math.PI * 1.0 / 4.0;

        public static double MODULE_DIST = Units.inchesToMeters(27.25 / 2.0);
        public static final Translation2d FL_POS = new Translation2d(MODULE_DIST, MODULE_DIST);
        public static final Translation2d FR_POS = new Translation2d(MODULE_DIST, -MODULE_DIST);
        public static final Translation2d BL_POS = new Translation2d(-MODULE_DIST, MODULE_DIST);
        public static final Translation2d BR_POS = new Translation2d(-MODULE_DIST, -MODULE_DIST);

        public static final Translation2d BLUE_SPEAKER_POS = new Translation2d(Units.inchesToMeters(-1.50 + 17.5),
                Units.inchesToMeters(218.42));
        public static final Translation2d RED_SPEAKER_POS = new Translation2d(Units.inchesToMeters(652.73 - 17.5),
                Units.inchesToMeters(218.42));
        public static final double SPEAKER_TO_SPEAKER = Units.inchesToMeters(651.23);
    }

    /** Constants for the intake subsystems. */
    public static class IntakeConstants {
        public static final int INTEGRATION_MOTOR_ID = 19;
        public static final int FRONT_MOTOR_ID = 17;
        public static final int BACK_MOTOR_ID = 18;
        public static final int PIVOT_MOTOR_ID = 16;

        public static final int frontSensorID = 0;
        public static final int backSensorID = 1;
        public static final int extendedlimitswitchID = 5;
        public static final int retractedlimitswitchID = 6;
        public static final int intakeencoderID = 3;

        public static final double encodermiddle = 1;
        public static final double rollersclockwise = 1;
        public static final double rollerscounterclockwise = 1;
        public static final double frontSensorReached = .05;
        public static final double BACK_SENSOR_REACHED = .5;
        public static final double pivotclockwise = 1;
        public static final double pivotcounterclockwise = -1;
        public static final double pastsensortime = 3;
    }

    /** Constants for Shooter Subsystem. */
    public static class ShooterConstants {
        //pivot constants
        public static final int LIMIT_SWITCH_ID = 4;
        public static final int PIVOT_MOTOR_ID = 12;
        public static final double CONVERSION_FACTOR = Units.degreesToRadians(44) / 6.33;
        public static final double PID_ERROR_TOLERANCE = Math.toRadians(2); //error tolerance for pid

        //flywheel constants
        public static final int SHOOTER_MOTOR_TOP_ID = 13;
        public static final int SHOOTER_MOTOR_BOTTOM_ID = 14;
        public static final double TOP_SHOOTER_MOTOR_SPEED = .6;
        public static final double BOTTOM_SHOOTER_MOTOR_SPEED = .64;

        //field constants
        //center of red speaker: (652.73 218.42)
        public static final double RED_X = Units.inchesToMeters(652.73 + 9.05);
        public static final double RED_Y = Units.inchesToMeters(218.42);

        //center of blue speaker: (-1.50 218.42)
        public static final double BLUE_X = Units.inchesToMeters(-1.5 + 9.05);
        public static final double BLUE_Y = Units.inchesToMeters(218.42);

        public static final double MAX_FLYWHEEL_RPS = 6380.0 / 60;
        public static final double MIN_SHOOTER_DISTANCE = 1.08;
        public static final double MAX_SHOOTER_DISTANCE = 8;


    }

    /** Constants for Climb Subsystem. */
    public static class ClimbConstants {
        public static final boolean CLIMB_IS_MANUAL = true;

        public static final int LEFT_SOLENOID_LATCH_PORT = 1;
        public static final int LEFT_WINCH_MOTOR_ID = 8;
        public static final int LEFT_ZERO_LIMIT_PORT = 8;

        public static final int RIGHT_SOLENOID_LATCH_PORT = 0;
        public static final int RIGHT_WINCH_MOTOR_ID = 9;
        public static final int RIGHT_ZERO_LIMIT_PORT = 3;
        
        /* As measured from bottom of base frame. */
        public static final double RAISE_LIMIT_METERS = Units.inchesToMeters(28);
        public static final double LOWER_LIMIT_METERS = Units.inchesToMeters(2);

        /* Find through empirical testing. */
        public static final double EXTENSION_METERS_PER_ROTATION = 0.60 / 103.2;
    }

    /** Constants for auto-aligning. */
    public static class AutoAlignConstants {

        public static final double robotRadius = (30 / 2) + 3.5;

        // X: APRILTAG_POS + SUBWOOFER_OFFSET + ROBOT_X_CENTER
        // ROT: rotated 180 deg from Apriltag pos since robots shoots from back
        public static final Pose2d BLUE_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(-1.50 + 37.711 + robotRadius),
                Units.inchesToMeters(218.42), Rotation2d.fromDegrees(0 + 180));

        // Source position closer to the centerline
        // Y: APRILTAG_POS
        public static final Pose2d BLUE_SOURCE_POSE = new Pose2d(Units.inchesToMeters(593.68),
                Units.inchesToMeters(9.68), Rotation2d.fromDegrees(120));

        public static final Pose2d BLUE_AMP_POSE = new Pose2d(Units.inchesToMeters(72.5),
                Units.inchesToMeters(323.00 - robotRadius), Rotation2d.fromDegrees(90));
        // public static final Pose2d BLUE_AMP_POSE = new
        // Pose2d(Units.inchesToMeters(323.00 - robotRadius - 11.0),
        // Units.inchesToMeters(72.5), Rotation2d.fromDegrees(90));
        public static final Pose2d ORIGIN_POSE = new Pose2d(0, 0, new Rotation2d(0));

        // X: APRILTAG_POS + SUBWOOFER_OFFSET + ROBOT_X_CENTER
        // ROT: rotated 180 deg from Apriltag pos since robots shoots from back
        public static final Pose2d RED_SPEAKER_POSE = new Pose2d(Units.inchesToMeters(652.73 - 37.711 - robotRadius),
                Units.inchesToMeters(218.42), Rotation2d.fromDegrees(180 - 180));

        // Source position closer to the centerline
        // Y: APRILTAG_POS
        public static final Pose2d RED_SOURCE_POSE = new Pose2d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68),
                Rotation2d.fromDegrees(60));

        public static final Pose2d RED_AMP_POSE = new Pose2d(Units.inchesToMeters(578.77),
                Units.inchesToMeters(323.00 - robotRadius), Rotation2d.fromDegrees(270));
    }

    /** Constants for the LED subsystem. */
    public static class LEDConstants {
        public static final int LED_LENGTH = 140;
        public static final int LED_PWM_PORT = 0;
        public static final double BRIGHTNESS_SCALE_FACTOR = .6;
    }

    /** Constants for vision. */
    public static final class VisionConstants {
        public static final String VISION_TABLE_KEY = "Vision";

        public static final PhotonCamera FRONT_CAMERA = new PhotonCamera("Arducam_OV9281_USB_Camera_2");
        public static final Transform3d FRONT_CAMERA_POSE = new Transform3d(
                new Translation3d(Units.inchesToMeters(+8), Units.inchesToMeters(4), Units.inchesToMeters(+44)),
                new Rotation3d(Math.PI, Units.degreesToRadians(16), 0));

        public static final PhotonCamera FRONT_RIGHT_CAMERA = new PhotonCamera("front right");
        public static final Transform3d FRONT_RIGHT_CAMERA_POSE = new Transform3d(
            new Translation3d(Units.inchesToMeters(+9.7), Units.inchesToMeters(-14.2), Units.inchesToMeters(+25.9)),
            new Rotation3d(Units.degreesToRadians(.3), Units.degreesToRadians(-16), Units.degreesToRadians(-20.2))
        );

        public static final PhotonCamera BACK_LEFT_CAMERA = new PhotonCamera("back left");
        public static final Transform3d  BACK_LEFT_CAMERA_POSE = new Transform3d(
            new Translation3d(Units.inchesToMeters(+3.9), Units.inchesToMeters(+14.5), Units.inchesToMeters(+26.3)),
            new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-16.0), Units.degreesToRadians(+160.0))
        );

        public static final PhotonCamera BACK_RIGHT_CAMERA = new PhotonCamera("back right");
        public static final Transform3d  BACK_RIGHT_CAMERA_POSE = new Transform3d(
            new Translation3d(Units.inchesToMeters(-5.1), Units.inchesToMeters(-11.6), Units.inchesToMeters(+22.3)),
            new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-20.0), Units.degreesToRadians(-170.0))
        );

        public static final PhotonCamera NOTE_CAMERA = new PhotonCamera("Microsoft_LifeCam_HD-3000");
        public static final Transform3d NOTE_CAMERA_POSE = new Transform3d();
    }
}
