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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeMouthConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.RobotState.IntakeState;
import frc.robot.subsystems.SubsystemInterfaces.AlgaeMouthSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GenericDirection;

public class AlgaeMouthSubsystem extends SubsystemBase implements AlgaeMouthSubsystemInterface {
    private static final class FakeAlgaeMouthSubsystem implements AlgaeMouthSubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }

        @Override
        public Command getManualArmOutCommand() {
            return Commands.none();
        }
        @Override
        public Command getManualArmInCommand() {
            return Commands.none();
        }
    }

    private static AlgaeMouthSubsystemInterface singleton = null;

    public static synchronized AlgaeMouthSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = AlgaeMouthConstants.REAL ? new AlgaeMouthSubsystem() : new FakeAlgaeMouthSubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private GenericDirection m_manualArmDirection = GenericDirection.NONE;
    private void setManualArmDirection(GenericDirection manualArmDirection) {
        m_manualArmDirection = manualArmDirection;
    }

    private DesiredControlType m_desiredArmControlType = DesiredControlType.AUTOMATIC;
    private void setDesiredArmControlType(DesiredControlType desiredArmControlType) {
        m_desiredArmControlType = desiredArmControlType;
    }

    private final TalonFX m_armMotor = new TalonFX(AlgaeMouthConstants.ARM_MOTOR_ID);
    private final TalonFX m_scoreMotor = new TalonFX(AlgaeMouthConstants.SCORE_MOTOR_ID);

    private final DutyCycleOut m_armDutyCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut m_scoreDutyCycleOut = new DutyCycleOut(0);
    // private final MotionMagicVoltage m_armPositionControl = new MotionMagicVoltage(0);
    private final NeutralOut m_brake = new NeutralOut();

    public AlgaeMouthSubsystem() {
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();

        armMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // motorConfigs.MotorOutput.PeakForwardDutyCycle = AlgaeMouthConstants.MAX_DUTY_CYCLE;
        // motorConfigs.MotorOutput.PeakReverseDutyCycle = -AlgaeMouthConstants.MAX_DUTY_CYCLE;

        // motorConfigs.CurrentLimits.StatorCurrentLimit = 25;
        // motorConfigs.CurrentLimits.SupplyCurrentLimit = 2.5;

        armMotorConfig.Feedback.SensorToMechanismRatio = (54d / 11d) * (54d / 18d) * (36d / 18d);

        // armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        // armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = AlgaeMouthConstants.MAX_ARM_POSITION_ROTATIONS;
        // armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = AlgaeMouthConstants.MIN_ARM_POSITION_ROTATIONS;

        OurUtils.tryApplyConfig(m_armMotor, armMotorConfig);

        TalonFXConfiguration scoreMotorConfig = new TalonFXConfiguration();

        scoreMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        scoreMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        scoreMotorConfig.MotorOutput.PeakForwardDutyCycle = AlgaeMouthConstants.SCORE_MAX_DUTY_CYCLE;
        scoreMotorConfig.MotorOutput.PeakReverseDutyCycle = -AlgaeMouthConstants.SCORE_MAX_DUTY_CYCLE;

        // scoreMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        // scoreMotorConfig.CurrentLimits.SupplyCurrentLimit = 80;
        scoreMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        scoreMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;

        OurUtils.tryApplyConfig(m_scoreMotor, scoreMotorConfig);

        m_armMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        handleArm();
        handleScoreMotor();
    }

    private void handleArm() {
        ControlRequest targetRequest = m_brake;

        switch (m_desiredArmControlType) {
            case AUTOMATIC:
                break;
            case MANUAL:
                switch (m_manualArmDirection) {
                    case NONE:
                        targetRequest = m_brake;
                        break;
                    case OUT:
                        targetRequest = m_armDutyCycleOut.withOutput(AlgaeMouthConstants.MANUAL_ARM_DUTY_CYCLE_OUT);
                        break;
                    case IN:
                        targetRequest = m_armDutyCycleOut.withOutput(AlgaeMouthConstants.MANUAL_ARM_DUTY_CYCLE_IN);
                        break;
                }
                break;
        }

        m_armMotor.setControl(targetRequest);
    }

    private void handleScoreMotor() {
        ControlRequest targetRequest = m_brake;

        if (RobotState.getWantsToScoreAlgae())
            targetRequest = m_scoreDutyCycleOut.withOutput(AlgaeMouthConstants.SCORE_MAX_DUTY_CYCLE);
        else if (RobotState.getIntakeState() == IntakeState.IN)
            targetRequest = m_scoreDutyCycleOut.withOutput(-AlgaeMouthConstants.SCORE_MAX_DUTY_CYCLE);

        m_scoreMotor.setControl(targetRequest);
    }

    private Command makeManualArmCommand(GenericDirection desiredDirection) {
        return startEnd(() -> {
            // switch (m_position) {
            //     case IDLE:
            //         break;
            //     default:
            //         setManualPosition(m_position.m_position);
            // }
            // setIdle();
            setManualArmDirection(desiredDirection);
            setDesiredArmControlType(DesiredControlType.MANUAL);
        }, () -> {
            setManualArmDirection(GenericDirection.NONE);
        });
    }
    private final Command m_manualArmOutCommand = makeManualArmCommand(GenericDirection.OUT);
    private final Command m_manualArmInCommand = makeManualArmCommand(GenericDirection.IN);

    @Override
    public Command getManualArmOutCommand() {
        return m_manualArmOutCommand;
    }

    @Override
    public Command getManualArmInCommand() {
        return m_manualArmInCommand;
    }
}
