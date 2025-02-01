// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.generated.TunerConstants;

public class Constants {
    public static final double ROBOT_MASS_KG = 38.782;
    public static final double ROBOT_MOMENT_OF_INTERTIA = 3.414;

    public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOMENT_OF_INTERTIA,
        new ModuleConfig(
            Meters.convertFrom(2.224, Inches),
            5.21208,
            1.542,
            DCMotor.getKrakenX60(4),
            TunerConstants.kSlipCurrent.in(Amp),
            4
        ),
        Meters.convertFrom(18.5, Inches)
    );

    public static final class ControllerConstants {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public static final class LEDConstants {
        // FIXME: LED CANDLE ID
        public static final int CANDLE_ID = -1;

        // FIXME: LED COUNT
        public static final int LED_COUNT = 47;

        public static final int COUNT_PER_GROUP = (int) Math.floor(LED_COUNT / 3);

        public static final int GROUP_1_START = 0;
        private static final int GROUP_1_END = COUNT_PER_GROUP;

        public static final int GROUP_2_START = GROUP_1_END + 1;
        private static final int GROUP_2_END = GROUP_2_START + GROUP_1_END;

        public static final int GROUP_3_START = GROUP_2_END + 1;
        // private static final int GROUP_3_END = LED_COUNT;
    }

    public static final class ElevatorConstants {
        // FIXME: ELEVATOR LEADER MOTOR ID
        public static final int LEADER_MOTOR_ID = -1;
        // FIXME: ELEVATOR FOLLOWER MOTOR ID
        public static final int FOLLOWER_MOTOR_ID = -1;

        // TODO: ideally, home and coral station are the same
        // FIXME: ELEVATOR HOME POSITION
        public static final double HOME_POSITION = 0;
        // FIXME: ELEVATOR CORAL STATION POSITION
        public static final double CORAL_STATION_POSITION = HOME_POSITION;

        // FIXME: ELEVATOR L1 POSITION
        public static final double L1_POSITION = HOME_POSITION;
        // FIXME: ELEVATOR L2 POSITION
        public static final double L2_POSITION = HOME_POSITION;
        // FIXME: ELEVATOR L3 POSITION
        public static final double L3_POSITION = HOME_POSITION;
        // FIXME: ELEVATOR L4 POSITION
        public static final double L4_POSITION = HOME_POSITION;

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
