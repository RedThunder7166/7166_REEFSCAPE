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

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GantryConstants;
import frc.robot.Constants.IntakeOuttakeConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface;

public class GantrySubsystem extends SubsystemBase implements GantrySubsystemInterface {
    private static final class FakeGantrySubsystem implements GantrySubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }

        @Override
        public void setAutomaticState(GantryState desiredState) { }

        @Override
        public void setIdle() { }

        @Override
        public void setManualDirection(GantryManualDirection desiredManualDirection) { }

        @Override
        public void incrementManualPosition(double value) { }

        @Override
        public void resetManualPosition() { }

        @Override
        public void setDesiredControlType(DESIRED_CONTROL_TYPE desiredControlType) { }

        @Override
        public void resetMotorPosition() { }

        @Override
        public boolean getIsAtTargetPosition() {
            return true;
        }

        @Override
        public Command getManualLeftCommand() {
            return Commands.none();
        }
        @Override
        public Command getManualRightCommand() {
            return Commands.none();
        }
    }

    private static GantrySubsystemInterface singleton = null;

    public static GantrySubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = GantryConstants.REAL ? new GantrySubsystem() : new FakeGantrySubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private GantryState m_state = GantryState.IDLE;
    @Override
    public void setAutomaticState(GantryState desiredState) {
        m_state = desiredState;
    }
    private final StringPublisher m_statePublisher = RobotState.m_robotStateTable.getStringTopic("GantryState").publish();

    @Override
    public void setIdle() {
        setAutomaticState(GantryState.IDLE);
        m_state = GantryState.IDLE;
        m_position = GantryPosition.IDLE;
    }

    private static enum GantryPosition {
        IDLE(GantryConstants.CORAL_STATION_POSITION), // m_position here should never be used, so we have it coral station to be safe
        CORAL_STATION(GantryConstants.CORAL_STATION_POSITION),
        REEF_LEFT(GantryConstants.REEF_LEFT_POSITION),
        REEF_RIGHT(GantryConstants.REEF_RIGHT_POSITION)

        ;
        private final double m_position;

        GantryPosition(double position) {
            m_position = position;
        }
    }
    private GantryPosition m_position = GantryPosition.IDLE;
    private void setAutomaticPosition(GantryPosition desiredPosition) {
        m_position = desiredPosition;
    }
    private final StringPublisher m_positionPublisher = RobotState.m_robotStateTable.getStringTopic("GantryPosition").publish();
    private final DoublePublisher m_automaticPositionRotationsPublisher = RobotState.m_robotStateTable.getDoubleTopic("GantryAutomaticPositionRotations").publish();

    private GantryManualDirection m_manualDirection = GantryManualDirection.NONE;
    @Override
    public void setManualDirection(GantryManualDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }
    private final StringPublisher m_manualDirectionPublisher = RobotState.m_robotStateTable.getStringTopic("GantryManualDirection").publish();

    private double m_manualPosition = 0;
    private void setManualPosition(double newValue) {
        if (newValue < GantryConstants.MIN_POSITION_ROTATIONS)
            newValue = GantryConstants.MIN_POSITION_ROTATIONS;
        if (newValue > GantryConstants.MAX_POSITION_ROTATIONS)
            newValue = GantryConstants.MAX_POSITION_ROTATIONS;

        m_manualPosition = newValue;
    }
    @Override
    public void incrementManualPosition(double value) {
        setManualPosition(m_manualPosition + value);
    }
    @Override
    public void resetManualPosition() {
        setManualPosition(0);
    }
    private final DoublePublisher m_manualPositionPublisher = RobotState.m_robotStateTable.getDoubleTopic("GantryManualTargetPosition").publish();

    private DESIRED_CONTROL_TYPE m_desiredControlType = DESIRED_CONTROL_TYPE.MANUAL;
    @Override
    public void setDesiredControlType(DESIRED_CONTROL_TYPE desiredControlType) {
        m_desiredControlType = desiredControlType;
    }
    private final StringPublisher m_desiredControlTypePublisher = RobotState.m_robotStateTable.getStringTopic("GantryDesiredControlType").publish();

    private final TalonFX m_gantryMotor = new TalonFX(GantryConstants.GANTRY_MOTOR_ID);
    private final TalonFX m_scoreMotor = new TalonFX(GantryConstants.SCORE_MOTOR_ID);

    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    private final StatusSignal<Angle> m_gantryMotorPosition = m_gantryMotor.getPosition();

    private final StatusSignal<Double> m_PIDPositionReference = m_gantryMotor.getClosedLoopReference();
    private final StatusSignal<Double> m_PIDPositionError = m_gantryMotor.getClosedLoopError();

    private final DoublePublisher m_gantryMotorPositionPublisher = RobotState.m_robotStateTable.getDoubleTopic("GantryMotorPosition").publish();
    private final DoublePublisher m_PIDPositionReferencePublisher = RobotState.m_robotStateTable.getDoubleTopic("GantryPIDPositionReferencePosition").publish();

    public GantrySubsystem() {
        // FIXME: tune gantry PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 0.7; // 4
        motorConfig.Slot0.kI = 0.05; // 0

        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 40;
        motionMagicConfigs.MotionMagicAcceleration = 120;

        OurUtils.tryApplyConfig(m_gantryMotor, motorConfig);

        TalonFXConfiguration scoreMotorConfig = new TalonFXConfiguration();

        OurUtils.tryApplyConfig(m_scoreMotor, scoreMotorConfig);

        // m_motor.setPosition(0);
    }

    @Override
    public void resetMotorPosition() {
        m_gantryMotor.setPosition(0);
    }

    @Override
    public boolean getIsAtTargetPosition() {
        // return true if we're not actively targeting a position
        if (m_desiredControlType == DESIRED_CONTROL_TYPE.AUTOMATIC && m_position == GantryPosition.IDLE)
            return true;

        double err = Math.abs(m_PIDPositionError.refresh().getValueAsDouble());
        SmartDashboard.putNumber("GantryPIDError", err);
        return Math.abs(m_PIDPositionError.refresh().getValueAsDouble()) <= GantryConstants.POSITION_ERROR_THRESHOLD;
    }
    private final BooleanPublisher m_isAtTargetPositionPublisher = RobotState.m_robotStateTable.getBooleanTopic("GantryIsAtTargetPosition").publish();

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
    private final Command m_manualLeftCommand = makeManualCommand(GantryManualDirection.LEFT);
    private final Command m_manualRightCommand = makeManualCommand(GantryManualDirection.RIGHT);

    @Override
    public Command getManualLeftCommand() {
        return m_manualLeftCommand;
    }
    @Override
    public Command getManualRightCommand() {
        return m_manualRightCommand;
    }

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

        handleScoreMotor();

        m_manualPositionPublisher.set(m_manualPosition);
        m_desiredControlTypePublisher.set(m_desiredControlType.toString());

        m_gantryMotorPosition.refresh();
        m_PIDPositionReference.refresh();

        m_gantryMotorPositionPublisher.set(m_gantryMotorPosition.getValueAsDouble());
        m_PIDPositionReferencePublisher.set(m_PIDPositionReference.getValueAsDouble());

        m_statePublisher.set(m_state.toString());
        m_positionPublisher.set(m_position.toString());
        m_manualDirectionPublisher.set(m_manualDirection.toString());

        m_isAtTargetPositionPublisher.set(getIsAtTargetPosition());
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
            case CORAL_STATION:
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

        if (desiredControl == m_positionControl)
            m_automaticPositionRotationsPublisher.set(m_positionControl.Position);

        m_gantryMotor.setControl(desiredControl);
    }
    private void handleManual() {
        final double increment = 0.5;

        switch (m_manualDirection) {
            case NONE:
                break;
            case LEFT:
                incrementManualPosition(increment);
                break;
            case RIGHT:
                incrementManualPosition(-increment);
                break;
        }

        m_gantryMotor.setControl(m_positionControl.withPosition(m_manualPosition));
        // m_motor.setControl(m_brake);
    }

    private void handleScoreMotor() {
        // TODO: score motor should not be controlled purely off intake
        ControlRequest targetRequest = m_brake;
        switch (RobotState.getIntakeState()) {
            case IDLE:
                targetRequest = m_brake; // redundant?
                break;
            case OUT:
                targetRequest = m_dutyCycleOut.withOutput(IntakeOuttakeConstants.BACKWARD_OUTPUT);
                break;
            case IN:
                targetRequest = m_dutyCycleOut.withOutput(IntakeOuttakeConstants.FORWARD_OUTPUT);
                break;
        }

        m_scoreMotor.setControl(targetRequest);
    }
}
