// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.RobotState.ClimbActuatorState;
import frc.robot.subsystems.SubsystemInterfaces.ClimbSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GenericDirection;

public class ClimbSubsystem extends SubsystemBase implements ClimbSubsystemInterface {
    private static final class FakeClimbSubsystem implements ClimbSubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }

        @Override
        public void incrementManualPosition(double value) { }

        @Override
        public void resetManualPosition() { }

        @Override
        public Command getManualActuatorOutCommand() {
            return Commands.none();
        }

        @Override
        public Command getManualActuatorInCommand() {
            return Commands.none();
        }

        @Override
        public Command getCageOutCommand() {
            return Commands.none();
        }

        @Override
        public Command getCageInCommand() {
            return Commands.none();
        }
    }

    private static ClimbSubsystemInterface singleton = null;

    public static ClimbSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = ClimbConstants.REAL ? new ClimbSubsystem() : new FakeClimbSubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private GenericDirection m_manualDirection = GenericDirection.NONE;
    public void setManualActuatorDirection(GenericDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }

    private double m_manualActuatorPosition = 0;
    private void setManualActuatorPosition(double newValue) {
        if (newValue < ClimbConstants.MIN_ACTUATOR_POSITION_ROTATIONS)
            newValue = ClimbConstants.MIN_ACTUATOR_POSITION_ROTATIONS;
        if (newValue > ClimbConstants.MAX_ACTUATOR_POSITION_ROTATIONS)
            newValue = ClimbConstants.MAX_ACTUATOR_POSITION_ROTATIONS;

        m_manualActuatorPosition = newValue;
    }
    @Override
    public void incrementManualPosition(double value) {
        setManualActuatorPosition(m_manualActuatorPosition + value);
    }
    @Override
    public void resetManualPosition() {
        setManualActuatorPosition(0);
    }

    private GenericDirection m_cageClimbDirection = GenericDirection.NONE;

    private final TalonFX m_actuatorMotor = new TalonFX(ClimbConstants.ACTUATOR_MOTOR_ID);
    private final TalonFX m_cageClimbMotor = new TalonFX(ClimbConstants.CAGE_CLIMB_MOTOR_ID, Constants.CANIVORE_NAME);

    private final MotionMagicVoltage m_actuatorPositionControl = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut m_cageDutyCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut m_actuatorDutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    // private final StatusSignal<Angle> m_actuatorMotorPosition = m_actuatorMotor.getPosition();
    // private final StatusSignal<Angle> m_cageClimbMotorPosition = m_cageClimbMotor.getPosition();

    public ClimbSubsystem() {
        TalonFXConfiguration actuatorConfigs = new TalonFXConfiguration();

        actuatorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        actuatorConfigs.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        actuatorConfigs.Slot0.kP = 450;

        actuatorConfigs.Feedback.SensorToMechanismRatio = (54d / 12d) * (54 / 18d) * (240d / 10d);

        actuatorConfigs.MotionMagic.MotionMagicAcceleration = 1;
        actuatorConfigs.MotionMagic.MotionMagicCruiseVelocity = 1;

        // not sure about these yet because the mechanism likes to skip
        // actuatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // actuatorConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.MIN_ACTUATOR_POSITION_ROTATIONS;
        // actuatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // actuatorConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.MAX_ACTUATOR_POSITION_ROTATIONS;

        OurUtils.tryApplyConfig(m_actuatorMotor, actuatorConfigs);

        TalonFXConfiguration cageClimbConfigs = new TalonFXConfiguration();

        cageClimbConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cageClimbConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        cageClimbConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cageClimbConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClimbConstants.MAX_CAGE_POSITION_ROTATIONS;
        cageClimbConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        cageClimbConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ClimbConstants.MIN_CAGE_POSITION_ROTATIONS;

        cageClimbConfigs.CurrentLimits.StatorCurrentLimit = 120d / 3;
        cageClimbConfigs.CurrentLimits.SupplyCurrentLimit = 40d / 3;

        OurUtils.tryApplyConfig(m_cageClimbMotor, cageClimbConfigs);

        m_actuatorMotor.setPosition(0);
        m_cageClimbMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        // m_actuatorMotorPosition.refresh();
        // m_cageClimbMotorPosition.refresh();

        handleActuator();
        handleCageClimb();
    }

    private void handleActuator() {
        ControlRequest targetRequest = m_brake;

        // final double cageClimbMotorPosition = m_cageClimbMotorPosition.getValueAsDouble();
        // final boolean canActuateIn = cageClimbMotorPosition >= ClimbConstants.MIN_SAFE_CAGE_POSITION_ROTATIONS;
        // final boolean canActuateOut = cageClimbMotorPosition <= ClimbConstants.MAX_SAFE_CAGE_POSITION_ROTATIONS;
        final boolean canActuateIn = true;
        final boolean canActuateOut = true;

        switch (RobotState.getClimbActuatorState()) {
            case MANUAL:
                final double amount = 0.05;
                double output = 0;
                switch (m_manualDirection) {
                    case NONE:
                        break;
                    case OUT:
                        if (canActuateOut)
                            output = amount;
                        break;
                    case IN:
                        if (canActuateIn)
                            output = -amount;
                        break;
                }
                targetRequest = m_actuatorDutyCycleOut.withOutput(output);
                break;
            case HOME:
                if (canActuateIn)
                    targetRequest = m_actuatorPositionControl.withPosition(ClimbConstants.MIN_ACTUATOR_POSITION_ROTATIONS);
                break;
            case CLIMB:
                if (canActuateOut)
                    targetRequest = m_actuatorPositionControl.withPosition(ClimbConstants.MAX_ACTUATOR_POSITION_ROTATIONS);
                break;
        }

        m_actuatorMotor.setControl(targetRequest);
    }
    private void handleCageClimb() {
        ControlRequest targetRequest = m_brake;

        // final double actuatorMotorPosition = m_actuatorMotorPosition.getValueAsDouble();
        // final boolean canGoIn = actuatorMotorPosition >= ClimbConstants.MIN_SAFE_ACTUATOR_POSITION_ROTATIONS;
        // final boolean canGoOut = actuatorMotorPosition <= ClimbConstants.MAX_SAFE_ACTUATOR_POSITION_ROTATIONS;
        final boolean canGoIn = true;
        final boolean canGoOut = true;

        switch (m_cageClimbDirection) {
            case NONE:
                targetRequest = m_brake;
                break;
            case OUT:
                if (canGoOut)
                    targetRequest = m_cageDutyCycleOut.withOutput(ClimbConstants.CAGE_DUTY_CYCLE_OUT);
                break;
            case IN:
                if (canGoIn)
                    targetRequest = m_cageDutyCycleOut.withOutput(ClimbConstants.CAGE_DUTY_CYCLE_IN);
                break;
        }

        m_cageClimbMotor.setControl(targetRequest);
    }

    private Command makeManualActuatorCommand(GenericDirection desiredDirection) {
        return new Command() {
            @Override
            public void initialize() {
                setManualActuatorDirection(desiredDirection);
                RobotState.setClimbActuatorState(ClimbActuatorState.MANUAL);
            }

            @Override
            public void end(boolean isInterrupted) {
                setManualActuatorDirection(GenericDirection.NONE);
            }
        };
    }
    private final Command m_manualActuatorOutCommand = makeManualActuatorCommand(GenericDirection.OUT);
    private final Command m_manualActuatorInCommand = makeManualActuatorCommand(GenericDirection.IN);

    @Override
    public Command getManualActuatorOutCommand() {
        return m_manualActuatorOutCommand;
    }
    @Override
    public Command getManualActuatorInCommand() {
        return m_manualActuatorInCommand;
    }

    private Command makeCageCommand(GenericDirection direction) {
        return new Command() {
            @Override
            public void initialize() {
                m_cageClimbDirection = direction;
            }

            @Override
            public void end(boolean isInterrupted) {
                m_cageClimbDirection = GenericDirection.NONE;
            }
        };
    }
    private final Command m_cageOutCommand = makeCageCommand(GenericDirection.OUT);
    private final Command m_cageInCommand = makeCageCommand(GenericDirection.IN);
    @Override
    public Command getCageOutCommand() {
        return m_cageOutCommand;
    }
    @Override
    public Command getCageInCommand() {
        return m_cageInCommand;
    }
}
