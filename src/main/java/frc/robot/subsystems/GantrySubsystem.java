// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GantryConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;

public class GantrySubsystem extends SubsystemBase {
    private static GantrySubsystem singleton = null;

    public static GantrySubsystem getSingleton() {
        if (singleton == null)
            singleton = new GantrySubsystem();
        return singleton;
    }

    public static enum GantryState {
        IDLE,
        SCORE,
        LOADING
    }
    private GantryState m_state = GantryState.IDLE;
    public void setAutomaticState(GantryState desiredState) {
        m_state = desiredState;
    }
    public void setIdle() {
        setAutomaticState(GantryState.IDLE);
        m_state = GantryState.IDLE;
        m_position = GantryPosition.IDLE;
    }

    private static enum GantryPosition {
        IDLE(GantryConstants.CORAL_STATION_POSITION),
        CORAL_STATION(GantryConstants.CORAL_STATION_POSITION), // // this position should never be used, so we have it coral station to be safe
        REEF_LEFT(GantryConstants.REEF_LEFT_POSITION),
        REEF_RIGHT(GantryConstants.REEF_RIGHT_POSITION)

        ;
        private final double m_position;

        GantryPosition(double position) {
            m_position = position;
        }
    }
    private GantryPosition m_position = GantryPosition.CORAL_STATION;
    private void setAutomaticPosition(GantryPosition desiredPosition) {
        m_position = desiredPosition;
    }

    public static enum GantryManualDirection {
        NONE,
        LEFT,
        RIGHT
    }
    private GantryManualDirection m_manualDirection = GantryManualDirection.NONE;
    public void setManualDirection(GantryManualDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }

        private double m_manualPosition = 0;
    private void setManualPosition(double newValue) {
        if (newValue < GantryConstants.MIN_POSITION_ROTATIONS)
            newValue = GantryConstants.MIN_POSITION_ROTATIONS;
        if (newValue > GantryConstants.MAX_POSITION_ROTATIONS)
            newValue = GantryConstants.MAX_POSITION_ROTATIONS;

        m_manualPosition = newValue;
    }
    public void incrementManualPosition(double value) {
        setManualPosition(m_manualPosition + value);
    }
    public void resetManualPosition() {
        setManualPosition(0);
    }

    private DESIRED_CONTROL_TYPE m_desiredControlType = DESIRED_CONTROL_TYPE.AUTOMATIC;
    public void setDesiredControlType(DESIRED_CONTROL_TYPE desiredControlType) {
        m_desiredControlType = desiredControlType;
    }

    private final TalonFX m_motor = new TalonFX(GantryConstants.MOTOR_ID);
    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    public GantrySubsystem() {
        // FIXME: tune gantry PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 16;

        // set Motion Magic settings
        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 40; // rps
        motionMagicConfigs.MotionMagicAcceleration = 120; // rps/s

        OurUtils.tryApplyConfig(m_motor, motorConfig);
    }

    private Command makeManualCommand(GantryManualDirection desiredDirection) {
        return startEnd(() -> {
            switch (m_position) {
                case IDLE:
                    break;
                default:
                    setManualPosition(m_position.m_position);
            }
            setDesiredControlType(DESIRED_CONTROL_TYPE.MANUAL);
            setIdle();
            setManualDirection(desiredDirection);
        }, () -> {
            setManualDirection(GantryManualDirection.NONE);
        });
    }
    public final Command m_manualLeftCommand = makeManualCommand(GantryManualDirection.LEFT);
    public final Command m_manualRightCommand = makeManualCommand(GantryManualDirection.RIGHT);

    @Override
    public void periodic() {
        // looks ugly, but compiler optimizes nicely
        if (RobotState.ENABLE_AUTOMATIC_GANTRY_CONTROL) {
            if (m_desiredControlType == DESIRED_CONTROL_TYPE.AUTOMATIC)
                handleAutomatic();
            else
                handleManual();
        } else
            handleManual();
    }

    private void handleAutomatic() {
        switch (m_state) {
            case IDLE:
                setAutomaticPosition(GantryPosition.IDLE);
                break;
            case SCORE:
                switch (RobotState.getTargetScorePosition()) {
                    case NONE:
                        setAutomaticPosition(GantryPosition.IDLE);
                        break;

                    case L1:
                        // FIXME: GANTRY TROUGH POSITIONS
                        setAutomaticPosition(GantryPosition.CORAL_STATION);
                        break;

                    case L2_L:
                    case L3_L:
                    case L4_L:
                        setAutomaticPosition(GantryPosition.REEF_LEFT);
                        break;

                    case L2_R:
                    case L3_R:
                    case L4_R:
                        setAutomaticPosition(GantryPosition.REEF_RIGHT);
                        break;
                }
                break;
            case LOADING:
                setAutomaticPosition(GantryPosition.CORAL_STATION);
                break;
        }

        ControlRequest desiredControl = m_brake;

        switch (m_position) {
            case IDLE:
                desiredControl = m_brake; // redundant?
                break;
            default:
                desiredControl = m_positionControl.withPosition(m_position.m_position);
                break;
        }

        m_motor.setControl(desiredControl);
    }
    private void handleManual() {
        // final double increment = 0.05;

        // switch (m_manualDirection) {
        //     case NONE:
        //         break;
        //     case LEFT:
        //         incrementManualPosition(increment);
        //         break;
        //     case RIGHT:
        //         incrementManualPosition(-increment);
        //         break;
        // }

        // m_motor.setControl(m_positionControl.withPosition(m_manualPosition));
        m_motor.setControl(m_brake);
    }
}
