// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
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

    private final RobotState m_robotState = RobotState.getSingleton();

    private final TalonFX m_motor = new TalonFX(ElevatorConstants.MOTOR_ID);
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    public ElevatorSubsystem() {
        // FIXME: tune elevator PID
        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        configs.Slot0.kI = 0; // No output for integrated error
        configs.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        // Peak output of 8 V
        configs.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        OurUtils.tryApplyConfig(m_motor, configs);
    }

@Override
    public void periodic() {
        switch (m_state) {
            case HOME:
                setPosition(ElevatorPosition.HOME);
                break;
            case IDLE:
                setPosition(ElevatorPosition.IDLE);
                break;
            case SCORE:
                switch (m_robotState.getTargetScorePosition()) {
                    case NONE:
                        setPosition(ElevatorPosition.IDLE);
                        break;

                    case L1:
                        setPosition(ElevatorPosition.L1);
                        break;

                    case L2_L:
                    case L2_R:
                        setPosition(ElevatorPosition.L2);
                        break;

                    case L3_L:
                    case L3_R:
                        setPosition(ElevatorPosition.L3);
                        break;

                    case L4_L:
                    case L4_R:
                        setPosition(ElevatorPosition.L4);
                        break;
                }
                break;
            case CORAL_STATION:
                setPosition(ElevatorPosition.CORAL_STATION);
                break;
        }

        ControlRequest desiredControl = m_brake;

        switch (m_position) {
            case HOME:
                desiredControl = m_positionVoltage.withPosition(ElevatorConstants.HOME_POSITION);
                break;
            case IDLE:
                desiredControl = m_brake; // redundant?
                break;
            case CORAL_STATION:
                desiredControl = m_positionVoltage.withPosition(ElevatorConstants.CORAL_STATION_POSITION);
                break;

            case L1:
                desiredControl = m_positionVoltage.withPosition(ElevatorConstants.L1_POSITION);
                break;
            case L2:
                desiredControl = m_positionVoltage.withPosition(ElevatorConstants.L2_POSITION);
                break;
            case L3:
                desiredControl = m_positionVoltage.withPosition(ElevatorConstants.L3_POSITION);
                break;
            case L4:
                desiredControl = m_positionVoltage.withPosition(ElevatorConstants.L4_POSITION);
                break;
        }

        m_motor.setControl(desiredControl);
    }

    private void setPosition(ElevatorPosition desiredPosition) {
        m_position = desiredPosition;
    }
}
