// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.subsystems.CameraSubsystem;

public final class RobotState {
    // private static RobotState singleton = null;

    // public static RobotState getSingleton() {
    //     if (singleton == null)
    //         singleton = new RobotState();
    //     return singleton;
    // }

    public static final boolean ENABLE_AUTOMATIC_ELEVATOR_CONTROL = true;
    public static final boolean ENABLE_AUTOMATIC_GANTRY_CONTROL = false;

    public static final NetworkTableInstance NETWORK_TABLE_INSTANCE = NetworkTableInstance.getDefault();
    public static final NetworkTable m_robotStateTable = NETWORK_TABLE_INSTANCE.getTable("RobotState");

    public static enum RELATIVE_SCORE_POSITION {
        NONE,
        L1,

        L2_L,
        L2_R,

        L3_L,
        L3_R,

        L4_L,
        L4_R
    }

    private static RELATIVE_SCORE_POSITION m_targetScorePosition = RELATIVE_SCORE_POSITION.NONE;
    private static final StringPublisher m_targetScorePositionPublisher = m_robotStateTable.getStringTopic("TargetScorePosition").publish();

    public static RELATIVE_SCORE_POSITION getTargetScorePosition() {
        return m_targetScorePosition;
    }
    public static boolean setTargetScorePosition(RELATIVE_SCORE_POSITION desiredPosition) {
        // FIXME: ask subsystems if we can switch to this position; if we can't, don't set target position (or set it to NONE) and return false
        // ^ e.g: we are trying to score a piece, we are intaking a piece
        m_targetScorePosition = desiredPosition;
        return true;
    }

    public static enum DESIRED_CONTROL_TYPE {
        AUTOMATIC,
        MANUAL
    }

    public static enum INTAKE_STATE {
        IDLE,
        OUT,
        IN
    }

    private static INTAKE_STATE m_intakeState = INTAKE_STATE.IDLE;
    private static final StringPublisher m_intakeStatePublisher = m_robotStateTable.getStringTopic("IntakeState").publish();

    public static INTAKE_STATE getIntakeState() {
        return m_intakeState;
    }

    public static void startIntake(boolean isForward) {
        m_intakeState = isForward ? INTAKE_STATE.OUT : INTAKE_STATE.IN;
    }
    public static void stopIntake() {
        m_intakeState = INTAKE_STATE.IDLE;
    }

    private static CameraSubsystem m_cameraSubsystem = CameraSubsystem.getSingleton();
    public static boolean getCanMoveScoringMechanisms() {
        // TODO: sensor logic
        return true;
    }

    public static void updateNetworkTables() {
        m_targetScorePositionPublisher.set(m_targetScorePosition.toString());
        m_intakeStatePublisher.set(m_intakeState.toString());
    }
}
