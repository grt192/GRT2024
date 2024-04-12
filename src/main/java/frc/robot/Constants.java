// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
        public static final boolean LIMIT_SWITCH_ENABLED = true;

        public static final int EXTENSION_ID = 10;
        public static final int EXTENSION_FOLLOW_ID = 11;

        public static final float EXTENSION_LIMIT_METERS = 0;

        public static final int ZERO_LIMIT_ID = 7;

        public static final double ZERO_POSITION = 0;
        public static final double AMP_POSITION = .9;
        public static final double TRAP_POSITION = 1.;

        public static final double EXTENSION_P = 2.3;
        public static final double EXTENSION_I = 0.0035;
        public static final double EXTENSION_D = 0;
        public static final double EXTENSION_TOLERANCE = 0.008;

        public static final double POSITION_CONVERSION_FACTOR = 1 / 26.357; // Units.inchesToMeters(30.)/63.5;
        public static final double VELOCITY_CONVERSION_FACTOR = 1;

        public static final double DOWN_POWER = -0.001; //the motor power to make the elevator move down slowly
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
        public static final int FL_DRIVE = 8;
        public static final int FL_STEER = 1;
        public static final double FL_OFFSET = -.9201 - Math.PI / 2;

        public static final int FR_DRIVE = 2;
        public static final int FR_STEER = 3;
        public static final double FR_OFFSET = .6216 + Math.PI / 2;

        public static final int BL_DRIVE = 4;
        public static final int BL_STEER = 5;
        public static final double BL_OFFSET = .2301 - Math.PI / 2;

        public static final int BR_DRIVE = 6;
        public static final int BR_STEER = 7;
        public static final double BR_OFFSET = 2.828 + Math.PI / 2;

        public static double MODULE_DIST = Units.inchesToMeters(27.25 / 2.0);
        public static final Translation2d FL_POS = new Translation2d(MODULE_DIST, MODULE_DIST);
        public static final Translation2d FR_POS = new Translation2d(MODULE_DIST, -MODULE_DIST);
        public static final Translation2d BL_POS = new Translation2d(-MODULE_DIST, MODULE_DIST);
        public static final Translation2d BR_POS = new Translation2d(-MODULE_DIST, -MODULE_DIST);

        public static final Translation2d BLUE_SPEAKER_POS = new Translation2d(Units.inchesToMeters(-1.50 + 8),
                Units.inchesToMeters(218.42));
        public static final Translation2d RED_SPEAKER_POS = new Translation2d(Units.inchesToMeters(652.73 - 8),
                Units.inchesToMeters(218.42));
        public static final double SPEAKER_TO_SPEAKER = Units.inchesToMeters(651.23);
    }

    /** Constants for the intake subsystems. */
    public static class IntakeConstants {
        public static final int INTEGRATION_MOTOR_ID = 19;
        public static final int FRONT_MOTOR_ID = 17;
        public static final int BACK_MOTOR_ID = 18;
        public static final int PIVOT_MOTOR_ID = 16;
        
        public static final double PIVOT_P = 1;
        public static final double PIVOT_I = 0;
        public static final double PIVOT_D = 0;
        public static final double PIVOT_CONVERSION_FACTOR = 0.2142;

        public static final int FRONT_SENSOR_ID = 0;
        public static final int BACK_SENSOR_ID = 1;
        public static final int EXTENDED_LIMIT_SWITCH_ID = 5;
        public static final int RETRACTED_LIMIT_SWITCH_ID = 6;
        public static final int INTAKE_ENCODER_ID = 3;

        public static final double FRONT_SENSOR_THRESHOLD = .1;
        public static final double BACK_SENSOR_THRESHOLD = .5;
        public static final double COLOR_SENSOR_RED_THRESHOLD = 45;
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

        public static final double MAX_FLYWHEEL_RPS = 6380.0 / 60;
        public static final double MIN_SHOOTER_DISTANCE = 1.2;
        public static final double MAX_SHOOTER_DISTANCE = 7;
        public static final double FLYWHEEL_SHUTTLE_SPEED = 0.3;        
    }

    /** Constants for Climb Subsystem. */
    public static class ClimbConstants {
        public static final int LEFT_WINCH_MOTOR_ID = 20;
        public static final int LEFT_ZERO_LIMIT_PORT = 0;

        public static final int RIGHT_WINCH_MOTOR_ID = 9;
        public static final int RIGHT_ZERO_LIMIT_PORT = 1;
        
        /* As measured from the limit switch. */
        public static final double RAISE_LIMIT_METERS = Units.inchesToMeters(27.5);
        public static final double LOWER_LIMIT_METERS = Units.inchesToMeters(0);

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
                Units.inchesToMeters(323.00 - robotRadius), Rotation2d.fromDegrees(90));

        /* Stage poses such that the climb hooks are directly above the chains. Poses are defined in terms of the
         * corresponding apriltag's pose transformed by a constant distance to the chain. "Stage Left", in this case,
         * refers to the left third of the stage when facing the stage from the driver station. We are aware that this
         * naming scheme differs from the theatre convention. */
        private static final Transform2d STAGE_TAG_TO_ROBOT = new Transform2d(Units.inchesToMeters(17.0),
                                                                              Units.inchesToMeters(0),
                                                                              Rotation2d.fromDegrees(180));

        public static final Pose2d BLUE_STAGE_BACK_POSE = new Pose2d(Units.inchesToMeters(209.48),
                                                                    Units.inchesToMeters(161.62),
                                                                    Rotation2d.fromDegrees(0))
                                                                    .transformBy(STAGE_TAG_TO_ROBOT);

        public static final Pose2d BLUE_STAGE_LEFT_POSE = new Pose2d(Units.inchesToMeters(182.73),
                                                                    Units.inchesToMeters(177.10),
                                                                    Rotation2d.fromDegrees(120))
                                                                    .transformBy(STAGE_TAG_TO_ROBOT);

        public static final Pose2d BLUE_STAGE_RIGHT_POSE = new Pose2d(Units.inchesToMeters(182.73),
                                                                    Units.inchesToMeters(146.19),
                                                                    Rotation2d.fromDegrees(240))
                                                                    .transformBy(STAGE_TAG_TO_ROBOT);

        public static final Pose2d RED_STAGE_BACK_POSE = new Pose2d(Units.inchesToMeters(441.74),
                                                                    Units.inchesToMeters(161.62),
                                                                    Rotation2d.fromDegrees(180))
                                                                    .transformBy(STAGE_TAG_TO_ROBOT);

        public static final Pose2d RED_STAGE_LEFT_POSE = new Pose2d(Units.inchesToMeters(468.69),
                                                                    Units.inchesToMeters(146.19),
                                                                    Rotation2d.fromDegrees(300))
                                                                    .transformBy(STAGE_TAG_TO_ROBOT);

        public static final Pose2d RED_STAGE_RIGHT_POSE = new Pose2d(Units.inchesToMeters(468.69),
                                                                    Units.inchesToMeters(177.10),
                                                                    Rotation2d.fromDegrees(60))
                                                                    .transformBy(STAGE_TAG_TO_ROBOT);

    }

    /** Constants for auton. */
    public static class AutonConstants {
        public static double INTAKE_SWERVE_SPEED = .3;
        public static double INTAKE_SWERVE_TIME = 1;

        public static double SHOOT_READY_TIME = 3;
        public static double SHOOT_FEED_TIME = .5;

        public static Pose2d TOP_START_POSE = new Pose2d(new Translation2d(1.43, 7.01), new Rotation2d());
        public static Pose2d MIDDLE_START_POSE = new Pose2d(new Translation2d(1.389, 5.55), new Rotation2d());
        public static Pose2d BOTTOM_START_POSE = new Pose2d(new Translation2d(1.43, 4.11), new Rotation2d());
    }

    /** Constants for the LED subsystem. */
    public static class LEDConstants {
        public static final int LED_LENGTH = 22;
        public static final int LED_PWM_PORT = 9;
        public static final double BRIGHTNESS_SCALE_FACTOR = .6;
    }

    /** Constants for vision. */
    public static final class VisionConstants {
        public static final String VISION_TABLE_KEY = "Vision";
        public static final Boolean USE_REAL_CAMERA_POSES = true; 

        public static final PhotonCamera FRONT_RIGHT_CAMERA = new PhotonCamera("front right");
        public static final Transform3d FRONT_RIGHT_CAMERA_POSE = new Transform3d(
            new Translation3d(+0.238, -0.357, +0.662), /* in meters */
            new Rotation3d(new Quaternion(+0.01331, +0.98037, -0.09192, +0.17394)) /* W X Y Z */
        );

        public static final PhotonCamera BACK_LEFT_CAMERA = new PhotonCamera("back left");
        public static final Transform3d  BACK_LEFT_CAMERA_POSE = USE_REAL_CAMERA_POSES
            /* Calculated Real Pose */
            ? new Transform3d(
                new Translation3d(-0.120, +0.305, +0.572), /* in meters */
                new Rotation3d(new Quaternion(-0.11148, +0.15802, +0.98081, +0.02497)) /* W X Y Z */
            )
            /* CAD Pose */
            : new Transform3d(
                new Translation3d(-0.126, +0.298, +0.563), /* in meters */
                new Rotation3d(new Quaternion(-0.13815, +0.17069, +0.97523, +0.02661)) /* W X Y Z */
            );

        public static final PhotonCamera BACK_RIGHT_CAMERA = new PhotonCamera("back right");
        public static final Transform3d  BACK_RIGHT_CAMERA_POSE = USE_REAL_CAMERA_POSES
            /* Calculated Real Pose */
            ? new Transform3d(
                new Translation3d(-0.161, -0.334, +0.516), /* in meters */
                new Rotation3d(new Quaternion(-0.09005, +0.18402, +0.02004, +0.97858)) /* W X Y Z */
            )
            /* CAD Pose */
            : new Transform3d(
                new Translation3d(-0.130, -0.294, +0.567), /* in meters */
                new Rotation3d(new Quaternion(-0.09190, +0.17498, +0.01993, +0.98007)) /* W X Y Z */
            );

        public static final PhotonCamera NOTE_CAMERA = new PhotonCamera("cap");
    }
}