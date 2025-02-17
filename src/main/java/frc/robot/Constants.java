// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
    public static final String CANIVORE_NAME = "Canivore";
    public static final Alliance ALLIANCE = DriverStation.getAlliance().orElse(Alliance.Blue);

    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class ElevatorConstants {
        public static final boolean REAL = true;
        public static final int LEADER_MOTOR_ID = 31; // left
        public static final int FOLLOWER_MOTOR_ID = 32; // right

        public static final double MIN_POSITION_ROTATIONS = 0.005;
        public static final double MAX_POSITION_ROTATIONS = 4.3;

        // TODO: ideally, home and coral station are the same
        // FIXME: ELEVATOR HOME POSITION
        public static final double HOME_POSITION = MIN_POSITION_ROTATIONS;
        // FIXME: ELEVATOR CORAL STATION POSITION
        public static final double CORAL_STATION_POSITION = HOME_POSITION;

        public static final double L1_POSITION = HOME_POSITION;
        public static final double L2_POSITION = 0.71;
        public static final double L3_POSITION = 2.01;
        public static final double L4_POSITION = 4.05;

        // FIXME: ELEVATOR POSITION ERROR THRESHOLD
        public static final double POSITION_ERROR_THRESHOLD = 0.00006;
    }
    public static final class GantryConstants {
        public static final boolean REAL = false;
        public static final int GANTRY_MOTOR_ID = 51;
        public static final int SCORE_MOTOR_ID = 53;

        public static final double MIN_POSITION_ROTATIONS = -7.4;
        public static final double MAX_POSITION_ROTATIONS = 8.8;

        public static final double CORAL_STATION_POSITION = 0;
        public static final double REEF_LEFT_POSITION = MIN_POSITION_ROTATIONS;
        public static final double REEF_RIGHT_POSITION = MAX_POSITION_ROTATIONS;

        // FIXME: GANTRY POSITION ERROR THRESHOLD
        public static final double POSITION_ERROR_THRESHOLD = 0.02;
    }
    public static final class IntakeOuttakeConstants {
        public static final boolean REAL = true;
        public static final int INTAKE_MOTOR_ID = 52;

        // TODO: use MotionMagicVoltage with velocity
        public static final double FORWARD_OUTPUT = 0.4;
        public static final double BACKWARD_OUTPUT = -FORWARD_OUTPUT;
    }

    public static final class AprilTagConstants {
        public static final int REEF_AB_TAGID = ALLIANCE == Alliance.Blue ? 18 : 7;
        public static final int REEF_CD_TAGID = ALLIANCE == Alliance.Blue ? 19 : 8;
        public static final int REEF_EF_TAGID = ALLIANCE == Alliance.Blue ? 20 : 9;
        public static final int REEF_GH_TAGID = ALLIANCE == Alliance.Blue ? 21 : 10;
        public static final int REEF_IJ_TAGID = ALLIANCE == Alliance.Blue ? 22 : 11;
        public static final int REEF_KL_TAGID = ALLIANCE == Alliance.Blue ? 17 : 6;

        public static final int CORAL_STATION_LEFT_TAGID = ALLIANCE == Alliance.Blue ? 13 : 1;
        public static final int CORAL_STATION_RIGHT_TAGID = ALLIANCE == Alliance.Blue ? 12 : 2;

        // below values are in meters
        public static final double INSIDE_REEF_ZONE_THRESHOLD = 1.6;

        private static final double CORAL_STATION_OFFSET_HORIZONTAL = 0.3;
        private static final double CORAL_STATION_OFFSET_VERTICAL = 0.3;
        public static final Translation2d CORAL_STATION_LEFT_OFFSET = ALLIANCE == Alliance.Blue ?
            new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
            new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
        public static final Translation2d CORAL_STATION_RIGHT_OFFSET = ALLIANCE == Alliance.Blue ?
            new Translation2d(-CORAL_STATION_OFFSET_HORIZONTAL, -CORAL_STATION_OFFSET_VERTICAL) :
            new Translation2d(CORAL_STATION_OFFSET_HORIZONTAL, CORAL_STATION_OFFSET_VERTICAL);
    }

    // FIXME: AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS; should be slightly above actual time needed
    public static final double AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS = 3;
}
