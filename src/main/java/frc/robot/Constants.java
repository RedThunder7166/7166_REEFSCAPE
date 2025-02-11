// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public class Constants {
    public static final String CANIVORE_NAME = "Canivore";

    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class ElevatorConstants {
        public static final int LEADER_MOTOR_ID = 11; // left
        public static final int FOLLOWER_MOTOR_ID = 14; // right

        public static final double MIN_POSITION_ROTATIONS = 0.01; // essentially the error, since ideal min position is 0
        public static final double MAX_POSITION_ROTATIONS = 36;

        // TODO: ideally, home and coral station are the same
        // FIXME: ELEVATOR HOME POSITION
        public static final double HOME_POSITION = MIN_POSITION_ROTATIONS;
        // FIXME: ELEVATOR CORAL STATION POSITION
        public static final double CORAL_STATION_POSITION = HOME_POSITION;

        // FIXME: ELEVATOR L1 POSITION
        public static final double L1_POSITION = HOME_POSITION;
        // FIXME: ELEVATOR L2 POSITION
        public static final double L2_POSITION = HOME_POSITION + 3;
        // FIXME: ELEVATOR L3 POSITION
        public static final double L3_POSITION = HOME_POSITION + 6;
        // FIXME: ELEVATOR L4 POSITION
        public static final double L4_POSITION = HOME_POSITION + 9;

        // FIXME: ELEVATOR MANUAL VELOCITY
        public static final double MANUAL_VELOCITY_FORWARD_RPS = 1;
        public static final double MANUAL_VELOCITY_BACKWARD_RPS = -MANUAL_VELOCITY_FORWARD_RPS;
    }
    public static final class GantryConstants {
        // FIXME: GANTRY MOTOR ID
        public static final int MOTOR_ID = -1;

        // FIXME: GANTRY CORAL_STATION POSITION
        public static final double CORAL_STATION_POSITION = 0;
        // FIXME: GANTRY REEF_LEFT POSITION
        public static final double REEF_LEFT_POSITION = CORAL_STATION_POSITION;
        // FIXME: GANTRY REEF_RIGHT POSITION
        public static final double REEF_RIGHT_POSITION = CORAL_STATION_POSITION;
    }
    public static final class IntakeOuttakeConstants {
        // FIXME: INTAKEOUTTAKE MOTOR ID
        public static final int MOTOR_ID = -1;

        public static final double FORWARD_VELOCITY_RPS = 1;
        public static final double BACKWARD_VELOCITY_RPS = -FORWARD_VELOCITY_RPS;
    }
}
