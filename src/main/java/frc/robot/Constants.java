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
        public static final int LEADER_MOTOR_ID = 31; // left
        public static final int FOLLOWER_MOTOR_ID = 32; // right

        public static final double MIN_POSITION_ROTATIONS = 0.01; // essentially the error, since ideal min position is 0
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

        // FIXME: ELEVATOR MANUAL VELOCITY
        public static final double MANUAL_VELOCITY_FORWARD_RPS = 1;
        public static final double MANUAL_VELOCITY_BACKWARD_RPS = -MANUAL_VELOCITY_FORWARD_RPS;
    }
    public static final class GantryConstants {
        public static final int MOTOR_ID = 51;

        public static final double MIN_POSITION_ROTATIONS = -7.4;
        public static final double MAX_POSITION_ROTATIONS = 8.8;

        public static final double CORAL_STATION_POSITION = 0;
        public static final double REEF_LEFT_POSITION = MIN_POSITION_ROTATIONS;
        public static final double REEF_RIGHT_POSITION = MAX_POSITION_ROTATIONS;
    }
    public static final class IntakeOuttakeConstants {
        public static final int SCORE_MOTOR_ID = 53;
        public static final int INTAKE_MOTOR_ID = 52;

        // TODO: use MotionMagicVoltage with velocity
        public static final double FORWARD_OUTPUT = 0.2;
        public static final double BACKWARD_OUTPUT = -FORWARD_OUTPUT;
    }
}
