// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GantryConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorManualDirection;

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

    private static enum GantryPosition {
        IDLE,
        CORAL_STATION, // aka loading
        REEF_LEFT,
        REEF_RIGHT
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
    public void setManualDirecion(GantryManualDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }

    private final TalonFX m_motor = new TalonFX(GantryConstants.MOTOR_ID);
    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    public GantrySubsystem() {
        // FIXME: tune gantry PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        motorConfig.Slot0.kI = 0; // No output for integrated error
        motorConfig.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        // Peak output of 8 V
        motorConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        // FIXME: tune gantry Motion Magic
        // set Motion Magic settings
        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // rps/s
        motionMagicConfigs.MotionMagicJerk = 1600; // rps/s/s

        OurUtils.tryApplyConfig(m_motor, motorConfig);
    }

    @Override
    public void periodic() {
        if (m_manualDirection == GantryManualDirection.NONE)
            handleAutomatic();
        else
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
            case CORAL_STATION:
                desiredControl = m_positionControl.withPosition(GantryConstants.CORAL_STATION_POSITION);
                break;
            case REEF_LEFT:
                desiredControl = m_positionControl.withPosition(GantryConstants.REEF_LEFT_POSITION);
                break;
            case REEF_RIGHT:
                desiredControl = m_positionControl.withPosition(GantryConstants.REEF_RIGHT_POSITION);
                break;
        }

        m_motor.setControl(desiredControl);
    }
    private void handleManual() {

    }
}
