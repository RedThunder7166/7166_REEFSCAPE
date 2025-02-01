// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState;

public class ElevatorSubsystem extends SubsystemBase {
    private static ElevatorSubsystem singleton = null;

    public static ElevatorSubsystem getSingleton() {
        if (singleton == null)
            singleton = new ElevatorSubsystem();
        return singleton;
    }

    public static enum ElevatorState {
        HOME,
        IDLE,
        SCORE,
        CORAL_STATION
    }
    private ElevatorState m_state = ElevatorState.HOME;
    private ElevatorState m_desiredState = ElevatorState.HOME;
    public void setAutomaticState(ElevatorState desiredState) {
        m_desiredState = desiredState;
    }

    private static enum ElevatorPosition {
        HOME,
        IDLE,
        CORAL_STATION,

        L1,
        L2,
        L3,
        L4,
    }
    private ElevatorPosition m_position = ElevatorPosition.HOME;
    private void setAutomaticPosition(ElevatorPosition desiredPosition) {
        m_position = desiredPosition;
    }

    public static enum ElevatorManualDirection {
        NONE,
        UP,
        DOWN
    }
    private ElevatorManualDirection m_manualDirection = ElevatorManualDirection.NONE;
    public void setManualDirecion(ElevatorManualDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }

    private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_ID);
    private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);
    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    public ElevatorSubsystem() {
        // FIXME: tune elevator PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        motorConfig.Slot0.kI = 0; // No output for integrated error
        motorConfig.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        // Peak output of 8 V
        motorConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        // FIXME: tune elevator Motion Magic; may be different per motor?
        // set Motion Magic settings
        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // rps/s
        motionMagicConfigs.MotionMagicJerk = 1600; // rps/s/s

        OurUtils.tryApplyConfig(m_leaderMotor, motorConfig);
        OurUtils.tryApplyConfig(m_followerMotor, motorConfig);

        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), false));
    }

    @Override
    public void periodic() {
        if (m_manualDirection == ElevatorManualDirection.NONE)
            handleAutomatic();
        else
            handleManual();
    }

    private void handleAutomatic() {
        // TODO: verify this should be IDLE
        m_state = RobotState.getCanMoveScoringMechanisms() ? m_desiredState : ElevatorState.IDLE;

        switch (m_state) {
            case HOME:
                setAutomaticPosition(ElevatorPosition.HOME);
                break;
            case IDLE:
                setAutomaticPosition(ElevatorPosition.IDLE);
                break;
            case SCORE:
                switch (RobotState.getTargetScorePosition()) {
                    case NONE:
                        setAutomaticPosition(ElevatorPosition.IDLE);
                        break;

                    case L1:
                        setAutomaticPosition(ElevatorPosition.L1);
                        break;

                    case L2_L:
                    case L2_R:
                        setAutomaticPosition(ElevatorPosition.L2);
                        break;

                    case L3_L:
                    case L3_R:
                        setAutomaticPosition(ElevatorPosition.L3);
                        break;

                    case L4_L:
                    case L4_R:
                        setAutomaticPosition(ElevatorPosition.L4);
                        break;
                }
                break;
            case CORAL_STATION:
                setAutomaticPosition(ElevatorPosition.CORAL_STATION);
                break;
        }

        ControlRequest desiredControl = m_brake;

        switch (m_position) {
            case HOME:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.HOME_POSITION);
                break;
            case IDLE:
                desiredControl = m_brake; // redundant?
                break;
            case CORAL_STATION:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.CORAL_STATION_POSITION);
                break;

            case L1:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L1_POSITION);
                break;
            case L2:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L2_POSITION);
                break;
            case L3:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L3_POSITION);
                break;
            case L4:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L4_POSITION);
                break;
        }

        m_leaderMotor.setControl(desiredControl);
    }
    private void handleManual() {
        ControlRequest desiredControl = m_brake;

        switch (m_manualDirection) {
            case NONE:
                desiredControl = m_brake; // redundant?
                break;
            case UP:
                desiredControl = m_velocityControl.withVelocity(ElevatorConstants.MANUAL_VELOCITY_FORWARD_RPS);
                break;
            case DOWN:
                desiredControl = m_velocityControl.withVelocity(ElevatorConstants.MANUAL_VELOCITY_BACKWARD_RPS);
                break;
        }

        m_leaderMotor.setControl(desiredControl);
    }
}
