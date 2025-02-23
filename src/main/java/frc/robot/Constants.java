// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final String CANIVORE_NAME = "Canivore";

    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class ElevatorConstants {
        public static final boolean REAL = true;
        public static final int LEADER_MOTOR_ID = 31; // left
        public static final int FOLLOWER_MOTOR_ID = 32; // right

        public static final double MIN_POSITION_ROTATIONS = 0.01;
        public static final double MAX_POSITION_ROTATIONS = 4.50;

        // TODO: ideally, home and coral station are the same
        // FIXME: ELEVATOR HOME POSITION
        public static final double HOME_POSITION = MIN_POSITION_ROTATIONS;
        // FIXME: ELEVATOR CORAL STATION POSITION
        public static final double CORAL_STATION_POSITION = HOME_POSITION;

        public static final double L1_POSITION = HOME_POSITION;
        public static final double L2_POSITION = 0.8;
        public static final double L3_POSITION = 2.10;
        public static final double L4_POSITION = 4.35;

        // FIXME: ELEVATOR POSITION ERROR THRESHOLD
        public static final double POSITION_ERROR_THRESHOLD = 0.03;
    }
    public static final class GantryConstants {
        public static final boolean REAL = true;
        public static final int GANTRY_MOTOR_ID = 51;
        public static final int SCORE_MOTOR_ID = 53;

        public static final int ELEVATOR_CLEARANCE_SENSOR_ID = 2;
        public static final int SCORE_ENTER_SENSOR_ID = 0;
        public static final int SCORE_EXIT_SENSOR_ID = 1;

        public static final double UNIT_TO_METERS = (0.403225 / 20.247);
        public static final double METERS_TO_UNIT = 1 / UNIT_TO_METERS;

        public static final double MIN_POSITION_ROTATIONS = -9.595;
        public static final double MAX_POSITION_ROTATIONS = 8.797;
        // public static final double LEFT_RIGHT_OFFSET = MAX_POSITION_ROTATIONS + MIN_POSITION_ROTATIONS; // only works when max is + and min is -
        // public static final double LEFT_RIGHT_OFFSET = Units.inchesToMeters(2);
        public static final double LEFT_RIGHT_OFFSET = 0;

        public static final double CORAL_STATION_POSITION = 0;
        public static final double REEF_LEFT_POSITION = -8.220703125;
        public static final double REEF_RIGHT_POSITION = 8.279296875;

        // FIXME: GANTRY POSITION ERROR THRESHOLD
        public static final double POSITION_ERROR_THRESHOLD = 0.07;
    }
    public static final class IntakeOuttakeConstants {
        public static final boolean REAL = true;
        public static final int INTAKE_MOTOR_ID = 52;

        // note: positive (forward) means toward gantry

        // TODO: use MotionMagicVoltage with velocity
        public static final double FORWARD_OUTPUT = 0.4;
        public static final double BACKWARD_OUTPUT = -0.2;

        public static final double CRAWL_FORWARD_OUTPUT = 0.11;
        public static final double CRAWL_BACKWARD_OUTPUT = -CRAWL_FORWARD_OUTPUT;
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

        private static final double CORAL_STATION_OFFSET_HORIZONTAL = 0.3;
        private static final double CORAL_STATION_OFFSET_VERTICAL = 0.3;
        public static Translation2d CORAL_STATION_LEFT_OFFSET;
        public static Translation2d CORAL_STATION_RIGHT_OFFSET;

        public static void update(Alliance alliance) {
            REEF_AB_TAGID = alliance == Alliance.Blue ? 18 : 7;
            REEF_CD_TAGID = alliance == Alliance.Blue ? 19 : 8;
            REEF_EF_TAGID = alliance == Alliance.Blue ? 20 : 9;
            REEF_GH_TAGID = alliance == Alliance.Blue ? 21 : 10;
            REEF_IJ_TAGID = alliance == Alliance.Blue ? 22 : 11;
            REEF_KL_TAGID = alliance == Alliance.Blue ? 17 : 6;

            CORAL_STATION_LEFT_TAGID = alliance == Alliance.Blue ? 13 : 1;
            CORAL_STATION_RIGHT_TAGID = alliance == Alliance.Blue ? 12 : 2;

            CORAL_STATION_LEFT_OFFSET = alliance == Alliance.Blue ?
                new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
                new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
            CORAL_STATION_RIGHT_OFFSET = alliance == Alliance.Blue ?
                new Translation2d(-CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
                new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
        }
    }

    // FIXME: AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS; should be slightly above actual time needed
    public static final double AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS = 3;
}
