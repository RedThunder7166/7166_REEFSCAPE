// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static final String CANIVORE_NAME = "Canivore";

    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class LEDConstants {
        public static final boolean REAL = true;

        public static final int CANDLE_ID = 59;

        public static final int LED_START_OFFSET = 8;
        public static final int LED_COUNT = 36;
        public static final int LED_END = LED_START_OFFSET + LED_COUNT;

        public static final int LED_COUNT_HALVED = LED_COUNT / 2;
        public static final int GROUP_SIZE = LED_COUNT_HALVED / 2;

        public static final double BRIGHTNESS = 1;
        public static final double ANIMATION_SPEED = 0.22;

        public static final int PRIMARY_HUE = 255;
        public static final int SECONDARY_HUE = 0;

        public static final int TERTIARY_HUE = 170;
    }
    public static final int REEF_YAW_LINEUP_THRESHOLD_DEGREES = 3;

    public static final class DriveConstants {
        public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        public static final double TRACK_WIDTH_X = Units.inchesToMeters(22.5);
        public static final double TRACK_WIDTH_Y = TRACK_WIDTH_X;

        public static final double DRIVE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2d, TRACK_WIDTH_Y / 2d);

        // FIXME: TUNE THESE
        public static final PIDConstants PATH_PLANNER_TRANSLATION_PID = new PIDConstants(3, 0, 0);
        // public static final PIDConstants PATH_PLANNER_TRANSLATION_PID = new PIDConstants(3, 0, 0);
        public static final PIDConstants PATH_PLANNER_ROTATION_PID = new PIDConstants(7, 0, 0);
    }

    public static final class ElevatorConstants {
        public static final boolean REAL = true;
        public static final int LEADER_MOTOR_ID = 31; // left
        public static final int FOLLOWER_MOTOR_ID = 32; // right

        public static final double MIN_POSITION_ROTATIONS = 0;
        public static final double MAX_POSITION_ROTATIONS = 4.50;

        public static final double HOME_POSITION = MIN_POSITION_ROTATIONS;
        public static final double CORAL_STATION_POSITION = HOME_POSITION;

        // TODO: ensure within max and min
        public static final double L2_POSITION = 0.9;
        public static final double L1_POSITION = L2_POSITION;
        public static final double L3_POSITION = 2.21;
        public static final double L4_POSITION = 4.3;

        public static final double ALGAE_HAND_POSITION_OFFSET = 0.25;
        public static final double AUTO_ADJUST_OFFSET = MAX_POSITION_ROTATIONS - 4.3; // 4.3 is l4

        // FIXME: ELEVATOR POSITION ERROR THRESHOLD
        public static final double POSITION_ERROR_THRESHOLD = 0.05;

        public static final double MIN_FORWARD_DISTANCE = Units.inchesToMeters(17.2);
        public static final double MAX_FORWARD_DISTANCE = Units.inchesToMeters(22.52);
        public static final double calculateAutoAdjust(double forwardDistanceInches) {
            /*
                linear regression from an x-y table, based on l4

                x represents forward distance in inches
                y represents elevator position offset

                https://www.desmos.com/calculator/legao7xgjj
            */
            return (0.0375937d * forwardDistanceInches) - 0.646611d;
        }
    }
    public static final class GantryConstants {
        public static final boolean REAL = true;
        public static final int GANTRY_MOTOR_ID = 51;
        public static final int SCORE_MOTOR_ID = 53;
        public static final int GANTRY_LASER_ID = 57;
        public static final int GANTRY_CANRANGE_ID = 58;

        public static final int ELEVATOR_CLEARANCE_SENSOR_ID = 2;
        public static final int SCORE_ENTER_SENSOR_ID = 0;
        public static final int SCORE_EXIT_SENSOR_ID = 1;

        public static final double MIN_POSITION_ROTATIONS = 0;
        public static final double MAX_POSITION_ROTATIONS = 4.469482; // 4.500732421875; 4.467041

        public static final double MIN_POSITION_MM = 40; // 17
        public static final double MAX_POSITION_MM = 440; // 400; 500; 430;500

        public static final double UNIT_TO_METERS = (0.4064d / MAX_POSITION_ROTATIONS);
        public static final double METERS_TO_UNIT = 1d / UNIT_TO_METERS;

        public static double encoderUnitsToMeters(double encoderUnits) {
            return encoderUnits * UNIT_TO_METERS;
        }
        public static double encoderUnitsToMillieters(double encoderUnits) {
            return (encoderUnits * UNIT_TO_METERS) * 1000d;
        }
        public static double metersToEncoderUnits(double meters) {
            return meters * METERS_TO_UNIT;
        }
        public static double millimetersToEncoderUnits(double mm) {
            return metersToEncoderUnits(mm / 1000d);
        }

        // public static final double LEFT_RIGHT_OFFSET = Units.inchesToMeters(2);
        public static final double LEFT_RIGHT_OFFSET = 0;

        // TODO: ensure within max and min
        public static final double CORAL_STATION_POSITION_ROTATIONS = Units.inchesToMeters(8.17) * METERS_TO_UNIT; // 7.92; 8.17; 8
        public static final double REEF_LEFT_POSITION_ROTATIONS = Units.inchesToMeters(2) * METERS_TO_UNIT;
        public static final double REEF_RIGHT_POSITION_ROTATIONS = Units.inchesToMeters(14) * METERS_TO_UNIT;

        public static final double REEF_LEFT_POSITION_MM = 370; // 40; 482; 372; 450;420
        public static final double CORAL_STATION_POSITION_MM = 235; // 195; 200; 190; 210; 190
        public static final double REEF_RIGHT_POSITION_MM = 70; // 380; 35; 15; 28;50

        // public static final double POSITION_ERROR_THRESHOLD = 0.07;
        public static final double POSITION_ERROR_THRESHOLD_MM = 11; // 10; 2.5; 5; 8

        public static final double BACKWARD_OUTPUT = -0.2;

        public static final double CRAWL_FORWARD_OUTPUT = 0.13; // 0.08; 0.12; 0.15
        public static final double CRAWL_BACKWARD_OUTPUT = -0.18; // -0.10; -0.14; -0.16

        public static final double CRAWL_FORWARD_VOLTAGE = 1.5;
        public static final double CRAWL_BACKWARD_VOLTAGE = -CRAWL_FORWARD_VOLTAGE;

        public static final double SCORE_OUTPUT = 0.35; // 0.4; 0.35
        public static final double SLOW_SCORE_OUTPUT = 0.25;
    }
    public static final class IntakeOuttakeConstants {
        public static final boolean REAL = true;
        public static final int INTAKE_MOTOR_ID = 52;

        // note: positive (forward) means toward gantry

        // TODO: use MotionMagicVoltage with velocity
        public static final double FORWARD_OUTPUT = 0.4;
        public static final double BACKWARD_OUTPUT = -0.2;
    }

    public static final class ClimbConstants {
        public static final boolean REAL = true;

        public static final int ACTUATOR_MOTOR_ID = 54;
        public static final int CAGE_CLIMB_MOTOR_ID = 55;

        public static final double MIN_ACTUATOR_POSITION_ROTATIONS = 0;
        public static final double MAX_ACTUATOR_POSITION_ROTATIONS = 0.110;
        // TODO: ensure within max and min
        public static final double CLIMB_ACTUATOR_POSITION_ROTATIONS = MAX_ACTUATOR_POSITION_ROTATIONS;

        // public static final double MIN_SAFE_ACTUATOR_POSITION_ROTATIONS = 50;
        // public static final double MAX_SAFE_ACTUATOR_POSITION_ROTATIONS = 10;

        public static final double CAGE_DUTY_CYCLE_OUT = 1;
        public static final double CAGE_DUTY_CYCLE_IN = -CAGE_DUTY_CYCLE_OUT;

        public static final double MIN_CAGE_POSITION_ROTATIONS = 0;
        public static final double MAX_CAGE_POSITION_ROTATIONS = 238.5; // 79.5

        // public static final double MIN_SAFE_CAGE_POSITION_ROTATIONS = 0.09;
        // public static final double MAX_SAFE_CAGE_POSITION_ROTATIONS = 0.05;
    }

    public static final class AlgaeHandConstants {
        public static final boolean REAL = true;

        public static final int MOTOR_ID = 56;

        public static final double MIN_POSITION_ROTATIONS = 0;
        public static final double MAX_POSITION_ROTATIONS = 0.495;
        // public static final double MIDDLE_POSITION_ROTATIONS = 0.5;

        public static final double MAX_DUTY_CYCLE = 1;
    }

    public static final class AlgaeMouthConstants {
        public static final boolean REAL = true;
        // TODO: algae mouth constants

        public static final int ARM_MOTOR_ID = 56;
        public static final int SCORE_MOTOR_ID = 57;

        public static final double MIN_ARM_POSITION_ROTATIONS = 0;
        public static final double MAX_ARM_POSITION_ROTATIONS = 1;

        public static final double SCORE_MAX_DUTY_CYCLE = 1;

        public static final double MANUAL_ARM_DUTY_CYCLE_OUT = 0.1;
        public static final double MANUAL_ARM_DUTY_CYCLE_IN = -MANUAL_ARM_DUTY_CYCLE_OUT;
    }

    public static final class AprilTagConstants {
        public static int REEF_AB_TAGID;
        public static int REEF_CD_TAGID;
        public static int REEF_EF_TAGID;
        public static int REEF_GH_TAGID;
        public static int REEF_IJ_TAGID;
        public static int REEF_KL_TAGID;

        public static int CORAL_STATION_LEFT_TAGID;
        public static int CORAL_STATION_RIGHT_TAGID;

        // below values are in meters
        public static final double INSIDE_REEF_ZONE_THRESHOLD = 1.6;
        public static final double AUTO_ADJUST_THRESHOLD = 1.8;

        public static final double REEF_FORWARD_LED_THRESHOLD = 0.5;
        public static final double REEF_HORIZONTAL_LED_THRESHOLD = 0.01;

        public static void update(Alliance alliance) {
            REEF_AB_TAGID = alliance == Alliance.Blue ? 18 : 7;
            REEF_CD_TAGID = alliance == Alliance.Blue ? 17 : 8;
            REEF_EF_TAGID = alliance == Alliance.Blue ? 22 : 9;
            REEF_GH_TAGID = alliance == Alliance.Blue ? 21 : 10;
            REEF_IJ_TAGID = alliance == Alliance.Blue ? 20 : 11;
            REEF_KL_TAGID = alliance == Alliance.Blue ? 19 : 6;

            CORAL_STATION_LEFT_TAGID = alliance == Alliance.Blue ? 13 : 1;
            CORAL_STATION_RIGHT_TAGID = alliance == Alliance.Blue ? 12 : 2;
        }
    }

    public static final class AutoConstants {
        public static final double GO_TO_POSITION_DISTANCE_METERS = 2.4; // 2.6; 2.2; 2.7

        public static final double GO_TO_POSITION_TIMEOUT_SECONDS = 3.5; // 1.3; 1.5
        public static final double TIME_UNTIL_CORAL_IS_SCORED_SECONDS = 0.013; // 0.05; 0.35; 0.02
    }
}
