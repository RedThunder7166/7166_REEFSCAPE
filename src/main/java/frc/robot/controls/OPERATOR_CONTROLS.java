// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

public final class OPERATOR_CONTROLS {
    private static final CommandGenericHID operatorController = new CommandGenericHID(ControllerConstants.OPERATOR_PORT);

    public static final Trigger INTAKE_OUT = operatorController.button(1);
    public static final Trigger INTAKE_IN = operatorController.button(2);

    public static final Trigger POSITION_CORAL_STATION = operatorController.button(3);

    public static final Trigger SCORE_L1 = operatorController.button(4);
    public static final Trigger SCORE_L2_L = operatorController.button(5);
    public static final Trigger SCORE_L2_R = operatorController.button(6);
    public static final Trigger SCORE_L3_L = operatorController.button(7);
    public static final Trigger SCORE_L3_R = operatorController.button(8);
    public static final Trigger SCORE_L4_L = operatorController.button(9);
    public static final Trigger SCORE_L4_R = operatorController.button(10);

    public static final Trigger SCORE_PIECE = operatorController.button(11);

    public static final Trigger ELEVATOR_MANUAL_DOWN = operatorController.button(20);
    public static final Trigger ELEVATOR_MANUAL_UP = operatorController.button(21);

    public static final Trigger GANTRY_MANUAL_LEFT = operatorController.button(30);
    public static final Trigger GANTRY_MANUAL_RIGHT = operatorController.button(31);

    public static final Trigger GANTRY_RESET_POSITION = operatorController.button(13);

    public static final Trigger CLIMBER_OUT = operatorController.button(24);
    public static final Trigger CLIMBER_IN = operatorController.button(23);

    public static final Trigger CLIMBER_CLIMB = operatorController.button(27);
    public static final Trigger CLIMBER_HOME = operatorController.button(28);

    public static final Trigger CAGE_OUT = operatorController.button(16);
    public static final Trigger CAGE_IN = operatorController.button(17);

    public static final Trigger ALGAE_HAND_OUT = operatorController.button(18);
    public static final Trigger ALGAE_HAND_IN = operatorController.button(19);
}
