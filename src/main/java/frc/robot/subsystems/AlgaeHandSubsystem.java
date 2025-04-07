// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeHandConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.subsystems.SubsystemInterfaces.AlgaeHandSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GenericDirection;

public class AlgaeHandSubsystem extends SubsystemBase implements AlgaeHandSubsystemInterface {
    private static final class FakeAlgaeHandSubsystem implements AlgaeHandSubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }

        @Override
        public boolean isTargetingManualOut() {
            return false;
        }

        @Override
        public Command getManualOutCommand() {
            return Commands.none();
        }

        @Override
        public Command getManualInCommand() {
            return Commands.none();
        }

        @Override
        public Command getHomeCommand() {
            return Commands.none();
        }

        @Override
        public Command getExtendedCommand() {
            return Commands.none();
        }
    }

    private static AlgaeHandSubsystemInterface singleton = null;

    public static synchronized AlgaeHandSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = AlgaeHandConstants.REAL ? new AlgaeHandSubsystem() : new FakeAlgaeHandSubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private GenericDirection m_manualDirection = GenericDirection.NONE;

    private static enum AlgaeHandPosition {
        HOME(AlgaeHandConstants.MIN_POSITION_ROTATIONS),
        EXTENDED(AlgaeHandConstants.MAX_POSITION_ROTATIONS)

        ;
        private final double m_position;

        AlgaeHandPosition(double position) {
            m_position = position;
        }
    }
    private AlgaeHandPosition m_position = AlgaeHandPosition.HOME;

    private DesiredControlType m_desiredControlType = DesiredControlType.AUTOMATIC;

    private final TalonFX m_motor = new TalonFX(AlgaeHandConstants.MOTOR_ID);

    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0);
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    public AlgaeHandSubsystem() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        motorConfigs.Slot0.kP = 50;

        motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfigs.MotorOutput.PeakForwardDutyCycle = AlgaeHandConstants.MAX_DUTY_CYCLE;
        motorConfigs.MotorOutput.PeakReverseDutyCycle = -AlgaeHandConstants.MAX_DUTY_CYCLE;

        var motionMagicConfigs = motorConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 40;
        motionMagicConfigs.MotionMagicAcceleration = 120;

        motorConfigs.CurrentLimits.StatorCurrentLimit = 15;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 10;

        motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaeHandConstants.MAX_POSITION_ROTATIONS;
        motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaeHandConstants.MIN_POSITION_ROTATIONS;

        OurUtils.tryApplyConfig(m_motor, motorConfigs);

        m_motor.setPosition(0);
    }

    @Override
    public void periodic() {
        ControlRequest targetRequest = m_brake;

        switch (m_desiredControlType) {
            case MANUAL: {
                final double amount = AlgaeHandConstants.MAX_DUTY_CYCLE;
                final double output;
                switch (m_manualDirection) {
                    default: // NONE
                        output = 0;
                        break;
                    case OUT:
                        output = amount;
                        break;
                    case IN:
                        output = -amount;
                        break;
                }
                targetRequest = m_dutyCycleOut.withOutput(output);
                break;
            }
            case AUTOMATIC: {
                targetRequest = m_positionControl.withPosition(m_position.m_position);
                break;
            }
        }

        m_motor.setControl(targetRequest);
    }

    @Override
    public boolean isTargetingManualOut() {
        return m_manualDirection == GenericDirection.OUT;
    }

    private Command makeManualCommand(GenericDirection desiredDirection) {
        return Commands.startEnd(
            () -> {
                m_desiredControlType = DesiredControlType.MANUAL;
                m_manualDirection = desiredDirection;
            }, () -> {
                m_manualDirection = GenericDirection.NONE;
            },
            this
        );
    }
    private final Command m_manualOutCommand = makeManualCommand(GenericDirection.OUT);
    private final Command m_manualInCommand = makeManualCommand(GenericDirection.IN);

    @Override
    public synchronized Command getManualOutCommand() {
        return m_manualOutCommand;
    }
    @Override
    public synchronized Command getManualInCommand() {
        return m_manualInCommand;
    }

    @Override
    public synchronized Command getHomeCommand() {
        return Commands.runOnce(() -> {
            m_position = AlgaeHandPosition.HOME;
            m_desiredControlType = DesiredControlType.AUTOMATIC;
        }, this);
    }

    @Override
    public synchronized Command getExtendedCommand() {
        return Commands.runOnce(() -> {
            m_position = AlgaeHandPosition.EXTENDED;
            m_desiredControlType = DesiredControlType.AUTOMATIC;
        }, this);
    }
}
