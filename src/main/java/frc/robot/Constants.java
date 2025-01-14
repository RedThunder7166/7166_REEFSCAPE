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

/** Add your docs here. */
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
        // Meters.convertFrom(18.5, Inches),
        Meters.convertFrom(18.5, Inches)
    );

    public static final int CANDLE_ID = 20;
    public static final int CANDLE_LED_COUNT = 47;
}
