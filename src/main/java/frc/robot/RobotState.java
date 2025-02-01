// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class RobotState {
    // private static RobotState singleton = null;

    // public static RobotState getSingleton() {
    //     if (singleton == null)
    //         singleton = new RobotState();
    //     return singleton;
    // }

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

    public static RELATIVE_SCORE_POSITION getTargetScorePosition() {
        return m_targetScorePosition;
    }
    public static boolean setTargetScorePosition(RELATIVE_SCORE_POSITION desiredPosition) {
        // FIXME: ask subsystems if we can switch to this position; if we can't, don't set target position (or set it to NONE) and return false
        // ^ e.g: we are trying to score a piece, we are intaking a piece
        m_targetScorePosition = desiredPosition;
        return true;
    }

    public static enum INTAKE_STATE {
        IDLE,
        FORWARD,
        BACKWARD
    }

    private static INTAKE_STATE m_intakeState = INTAKE_STATE.IDLE;

    public static INTAKE_STATE getIntakeState() {
        return m_intakeState;
    }

    public static void startIntake(boolean isForward) {
        m_intakeState = isForward ? INTAKE_STATE.FORWARD : INTAKE_STATE.BACKWARD;
    }
    public static void stopIntake() {
        m_intakeState = INTAKE_STATE.IDLE;
    }
}
