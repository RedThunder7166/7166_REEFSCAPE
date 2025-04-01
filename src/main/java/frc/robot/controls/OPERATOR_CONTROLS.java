// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

public final class OPERATOR_CONTROLS {
    private static final CommandGenericHID operatorController = new CommandGenericHID(ControllerConstants.OPERATOR_PORT);

    public static final Trigger POSITION_CORAL_STATION = operatorController.button(1);

    public static final Trigger SCORE_L1 = operatorController.button(2);
    public static final Trigger SCORE_L2_L = operatorController.button(3);
    public static final Trigger SCORE_L2_R = operatorController.button(4);
    public static final Trigger SCORE_L3_L = operatorController.button(5);
    public static final Trigger SCORE_L3_R = operatorController.button(6);
    public static final Trigger SCORE_L4_L = operatorController.button(7);
    public static final Trigger SCORE_L4_R = operatorController.button(8);

    public static final Trigger INTAKE_OUT = operatorController.button(9);
    public static final Trigger INTAKE_IN = operatorController.button(10);

    public static final Trigger CORAL_SCORE = operatorController.button(11);
    public static final Trigger ALGAE_SCORE = operatorController.button(12);

    public static final Trigger MODE_ALGAE = operatorController.button(13);
    public static final Trigger MODE_CORAL = operatorController.button(14);

    public static final Trigger CLIMB_COMBO = operatorController.button(15);

    public static final Trigger ELEVATOR_MANUAL_DOWN = operatorController.button(20);
    public static final Trigger ELEVATOR_MANUAL_UP = operatorController.button(21);

    public static final Trigger GANTRY_MANUAL_LEFT = operatorController.button(22);
    public static final Trigger GANTRY_MANUAL_RIGHT = operatorController.button(23);

    public static final Trigger ALGAE_MANUAL_OUT = operatorController.button(24);
    public static final Trigger ALGAE_MANUAL_IN = operatorController.button(25);

    public static final Trigger CAGE_OUT = operatorController.button(26);
    public static final Trigger CAGE_IN = operatorController.button(27);

    public static final Trigger CLIMBER_OUT = operatorController.button(28);
    public static final Trigger CLIMBER_IN = operatorController.button(29);

    public static final Trigger CLIMBER_CLIMB = operatorController.button(30);
    public static final Trigger CLIMBER_HOME = operatorController.button(31);
}
