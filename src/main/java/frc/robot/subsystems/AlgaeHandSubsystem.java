// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeHandConstants;
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
        public Command getManualOutCommand() {
            return Commands.none();
        }

        @Override
        public Command getManualInCommand() {
            return Commands.none();
        }

        @Override
        public Command getMiddleCommand() {
            return Commands.none();
        }
    }

    private static AlgaeHandSubsystemInterface singleton = null;

    public static AlgaeHandSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = AlgaeHandConstants.REAL ? new AlgaeHandSubsystem() : new FakeAlgaeHandSubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private GenericDirection m_direction = GenericDirection.NONE;

    private DesiredControlType m_desiredControlType = DesiredControlType.AUTOMATIC;

    private final TalonFX m_motor = new TalonFX(AlgaeHandConstants.MOTOR_ID);

    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    public AlgaeHandSubsystem() {
        TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

        motorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfigs.MotorOutput.PeakForwardDutyCycle = AlgaeHandConstants.MAX_DUTY_CYCLE;
        motorConfigs.MotorOutput.PeakReverseDutyCycle = -AlgaeHandConstants.MAX_DUTY_CYCLE;

        motorConfigs.CurrentLimits.StatorCurrentLimit = 25;
        motorConfigs.CurrentLimits.SupplyCurrentLimit = 2.5;

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
                final double amount = 0.075;
                double output = 0;
                switch (m_direction) {
                    case NONE:
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
                targetRequest = m_brake;
                break;
            }
        }

        m_motor.setControl(targetRequest);
    }

    private Command makeManualCommand(GenericDirection desiredDirection) {
        return Commands.startEnd(() -> {
                m_desiredControlType = DesiredControlType.MANUAL;
                m_direction = desiredDirection;
            }, () -> {
                m_direction = GenericDirection.NONE;
            }, this
        );
    }
    private final Command m_manualOutCommand = makeManualCommand(GenericDirection.OUT);
    private final Command m_manualInCommand = makeManualCommand(GenericDirection.IN);

    @Override
    public Command getManualOutCommand() {
        return m_manualOutCommand;
    }
    @Override
    public Command getManualInCommand() {
        return m_manualInCommand;
    }

    // private final Command m_middleCommand = Commands.startEnd(() -> {
    //     m_desiredControlType = DesiredControlType.AUTOMATIC;
    // }, () -> {
        
    // }, this);
    private final Command m_middleCommand = new InstantCommand(() -> {
        m_desiredControlType = DesiredControlType.AUTOMATIC;
    }, this);

    @Override
    public Command getMiddleCommand() {
        return m_middleCommand;
    }
}
