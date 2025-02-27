// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.OurUtils;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.subsystems.Mechanisms.ElevatorMechanisms;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface;

public class ElevatorSubsystem extends SubsystemBase implements ElevatorSubsystemInterface {
    private static final class FakeElevatorSubsystem implements ElevatorSubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }

        @Override
        public void setAutomaticState(ElevatorState desiredState) { }

        @Override
        public void setIdle() { }

        @Override
        public void incrementManualPosition(double value) { }

        @Override
        public void resetManualPosition() { }

        @Override
        public void setDesiredControlType(DesiredControlType desiredControlType) { }

        @Override
        public boolean getIsAtTargetPosition() {
            return true;
        }

        @Override
        public Command getManualUpCommand() {
            return Commands.none();
        }
        @Override
        public Command getManualDownCommand() {
            return Commands.none();
        }

        @Override
        public Command getTempGoUntilTargetIncreaseCommand(double targetIncreaseInPosition) {
            return Commands.none();
        }

        @Override
        public Command getTempHoldPositionCommand() {
            return Commands.none();
        }

        @Override
        public Command getTimeTravelCommand(double targetPosition) {
            return Commands.none();
        }
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private static ElevatorSubsystemInterface singleton = null;

    public static ElevatorSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = ElevatorConstants.REAL ? new ElevatorSubsystem() : new FakeElevatorSubsystem();
        return singleton;
    }

    private ElevatorState m_state = ElevatorState.HOME;
    private ElevatorState m_desiredState = m_state;
    @Override
    public void setAutomaticState(ElevatorState desiredState) {
        m_desiredState = desiredState;
    }
    private final StringPublisher m_statePublisher = RobotState.robotStateTable.getStringTopic("ElevatorState").publish();
    private final StringPublisher m_desiredStatePublisher = RobotState.robotStateTable.getStringTopic("ElevatorDesiredState").publish();

    @Override
    public void setIdle() {
        setAutomaticState(ElevatorState.IDLE);
        m_state = ElevatorState.IDLE;
        m_position = ElevatorPosition.IDLE;
    }

    private static enum ElevatorPosition {
        HOME(ElevatorConstants.HOME_POSITION),
        IDLE(ElevatorConstants.HOME_POSITION), // this position should never be used, so we have it home to be safe
        CORAL_STATION(ElevatorConstants.CORAL_STATION_POSITION),

        L1(ElevatorConstants.L1_POSITION),
        L2(ElevatorConstants.L2_POSITION, true),
        L3(ElevatorConstants.L3_POSITION, true),
        L4(ElevatorConstants.L4_POSITION, true)

        ;
        private final double m_position;
        private final boolean m_needsElevatorClearance;

        ElevatorPosition(double position) {
            m_position = position;
            m_needsElevatorClearance = false;
        }
        ElevatorPosition(double position, boolean needsElevatorClearance) {
            m_position = position;
            m_needsElevatorClearance = needsElevatorClearance;
        }
    }
    private ElevatorPosition m_position = ElevatorPosition.IDLE;
    private void setAutomaticPosition(ElevatorPosition desiredPosition) {
        if (desiredPosition.m_needsElevatorClearance && !RobotState.getElevatorHasClearance())
            return;

        m_position = desiredPosition;
    }
    private final StringPublisher m_positionPublisher = RobotState.robotStateTable.getStringTopic("ElevatorPosition").publish();
    private final DoublePublisher m_automaticPositionRotationsPublisher = RobotState.robotStateTable.getDoubleTopic("ElevatorAutomaticPositionRotations").publish();

    private ElevatorManualDirection m_manualDirection = ElevatorManualDirection.NONE;
    public void setManualDirection(ElevatorManualDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }
    private final StringPublisher m_manualDirectionPublisher = RobotState.robotStateTable.getStringTopic("ElevatorManualDirection").publish();

    private double m_manualPosition = 0;
    private void setManualPosition(double newValue) {
        if (newValue < ElevatorConstants.MIN_POSITION_ROTATIONS)
            newValue = ElevatorConstants.MIN_POSITION_ROTATIONS;
        if (newValue > ElevatorConstants.MAX_POSITION_ROTATIONS)
            newValue = ElevatorConstants.MAX_POSITION_ROTATIONS;

        // if (!RobotState.getWristHasElevatorClearance() && newValue > ElevatorConstants.MAX_TARGET_POSITION_WITH_WRIST)
        //     newValue = ElevatorConstants.MAX_TARGET_POSITION_WITH_WRIST;

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
    private final DoublePublisher m_manualPositionPublisher = RobotState.robotStateTable.getDoubleTopic("ElevatorManualTargetPosition").publish();

    private DesiredControlType m_desiredControlType = DesiredControlType.AUTOMATIC;
    @Override
    public void setDesiredControlType(DesiredControlType desiredControlType) {
        m_desiredControlType = desiredControlType;
    }
    private final StringPublisher m_desiredControlTypePublisher = RobotState.robotStateTable.getStringTopic("ElevatorDesiredControlType").publish();

    private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_ID, Constants.CANIVORE_NAME);
    private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID, Constants.CANIVORE_NAME);

    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    private final StatusSignal<Angle> m_leaderMotorPosition = m_leaderMotor.getPosition();
    private final StatusSignal<Angle> m_followerMotorPosition = m_followerMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_leaderMotorVelocity = m_leaderMotor.getVelocity();

    private final StatusSignal<Double> m_PIDPositionReference = m_leaderMotor.getClosedLoopReference();
    private final StatusSignal<Double> m_PIDPositionSlope = m_leaderMotor.getClosedLoopReferenceSlope();
    private final StatusSignal<Double> m_PIDPositionError = m_leaderMotor.getClosedLoopError();

    private final DoublePublisher m_leaderMotorPositionPublisher = RobotState.robotStateTable.getDoubleTopic("ElevatorLeaderMotorPosition").publish();
    private final DoublePublisher m_followerMotorPositionPublisher = RobotState.robotStateTable.getDoubleTopic("ElevatorFollowerMotorPosition").publish();
    private final DoublePublisher m_leaderMotorVelocityPublisher = RobotState.robotStateTable.getDoubleTopic("ElevatorLeaderMotorVelocity").publish();

    private final DoublePublisher m_PIDPositionReferencePublisher = RobotState.robotStateTable.getDoubleTopic("ElevatorPIDPositionReference").publish();
    private final DoublePublisher m_PIDPositionSlopePublisher = RobotState.robotStateTable.getDoubleTopic("ElevatorPIDPositionSlope").publish();

    private static final boolean tuneWithNetworkTables = true;

    /* SOME NOTES
        2.4 inches per rotation of the motor
        11.81 inches per rotation of the gearbox (post gearbox)

        first iter: weights no gantry:
        kG = 0.32
        kV = 0.6
        kA = 0.03

        second iter:
        kG = 0.3
        kV = 0.82
        kA = 0.03
        kP = 3

        third iter:
        kG = 0.223
        kV = 0.67
        kA = 0.03
        kP = 10

        fourth iter:
        kG = 0.35
        kV = 0.67
        kA = 0.03
        kP = 10
     */
    private double m_kg = 0.35;
    private double m_kv = 0.67;
    private double m_ka = 0.03;

    private double m_ks = 0;

    private double m_kp = 10;
    private double m_ki = 0;
    private double m_kd = 0;

    // POSITION 2.036

    {
        if (tuneWithNetworkTables) {
            SmartDashboard.putNumber("ELEVATOR_KG", m_kg);
            SmartDashboard.putNumber("ELEVATOR_KV", m_kv);
            SmartDashboard.putNumber("ELEVATOR_KA", m_ka);

            SmartDashboard.putNumber("ELEVATOR_KS", m_ks);

            SmartDashboard.putNumber("ELEVATOR_KP", m_kp);
            SmartDashboard.putNumber("ELEVATOR_KI", m_ki);
            SmartDashboard.putNumber("ELEVATOR_KD", m_kd);
        }
    }

    public ElevatorSubsystem() {
        // FIXME: tune elevator PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        motorConfig.Slot0.kG = tuneWithNetworkTables ? SmartDashboard.getNumber("ELEVATOR_KG", m_kg) : m_kg;
        motorConfig.Slot0.kV = tuneWithNetworkTables ? SmartDashboard.getNumber("ELEVATOR_KV", m_kv) : m_kv;
        motorConfig.Slot0.kA = tuneWithNetworkTables ? SmartDashboard.getNumber("ELEVATOR_KA", m_ka) : m_ka;

        motorConfig.Slot0.kS = tuneWithNetworkTables ? SmartDashboard.getNumber("ELEVATOR_KS", m_ks) : m_ks;

        motorConfig.Slot0.kP = tuneWithNetworkTables ? SmartDashboard.getNumber("ELEVATOR_KP", m_kp) : m_kp;
        motorConfig.Slot0.kI = tuneWithNetworkTables ? SmartDashboard.getNumber("ELEVATOR_KI", m_ki) : m_ki;
        motorConfig.Slot0.kD = tuneWithNetworkTables ? SmartDashboard.getNumber("ELEVATOR_KD", m_kd) : m_kd;

        // TODO: once tuned, find peak voltage and set
        // motorConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
        //     .withPeakReverseVoltage(Volts.of(-8));

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.MAX_POSITION_ROTATIONS;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ElevatorConstants.MIN_POSITION_ROTATIONS;

        motorConfig.Feedback.SensorToMechanismRatio = 4.909;

        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        // FIXME: tune elevator Motion Magic; may be different per motor?
        var motionMagicConfigs = motorConfig.MotionMagic;

        var value = 15;
        motionMagicConfigs.MotionMagicCruiseVelocity = value;
        motionMagicConfigs.MotionMagicAcceleration = value;

        OurUtils.tryApplyConfig(m_leaderMotor, motorConfig);
        OurUtils.tryApplyConfig(m_followerMotor, motorConfig);

        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), false));

        m_leaderMotor.setPosition(0);
        m_followerMotor.setPosition(0);

        SmartDashboard.putData("ElevatorMech2d", ElevatorMechanisms.mechanism2d);
    }

    // TODO: this should be called on sensor trip
    private void resetMotorPositions() {
        m_leaderMotor.setPosition(0);
        m_followerMotor.setPosition(0);
    }

    @Override
    public boolean getIsAtTargetPosition() {
        // return true if we're not actively targeting a position
        if (m_desiredControlType == DesiredControlType.AUTOMATIC && m_position == ElevatorPosition.IDLE)
            return true;

        // double err = Math.abs(m_PIDPositionError.refresh().getValueAsDouble());
        final double err = Math.abs(m_leaderMotorPosition.getValueAsDouble() - m_position.m_position);
        SmartDashboard.putNumber("ElevatorPIDError", err);
        return err <= ElevatorConstants.POSITION_ERROR_THRESHOLD;
    }
    private final BooleanPublisher m_isAtTargetPositionPublisher = RobotState.robotStateTable.getBooleanTopic("ElevatorIsAtTargetPosition").publish();

    private Command makeManualCommand(ElevatorManualDirection desiredDirection) {
        return startEnd(() -> {
            switch (m_position) {
                case IDLE:
                    break;
                default:
                    setManualPosition(m_position.m_position);
            }
            setDesiredControlType(DesiredControlType.MANUAL);
            setIdle();
            setManualDirection(desiredDirection);
        }, () -> {
            setManualDirection(ElevatorManualDirection.NONE);
        });
    }
    private final Command m_manualUpCommand = makeManualCommand(ElevatorManualDirection.UP);
    private final Command m_manualDownCommand = makeManualCommand(ElevatorManualDirection.DOWN);

    @Override
    public Command getManualUpCommand() {
        return m_manualUpCommand;
    }
    @Override
    public Command getManualDownCommand() {
        return m_manualDownCommand;
    }

    @Override
    public void periodic() {
        if (tuneWithNetworkTables) {
            var kG = SmartDashboard.getNumber("ELEVATOR_KG", m_kg);
            var kV = SmartDashboard.getNumber("ELEVATOR_KV", m_kv);
            var kA = SmartDashboard.getNumber("ELEVATOR_KA", m_ka);

            var kS = SmartDashboard.getNumber("ELEVATOR_KS", m_ks);

            var kP = SmartDashboard.getNumber("ELEVATOR_KP", m_kp);
            var kI = SmartDashboard.getNumber("ELEVATOR_KI", m_ki);
            var kD = SmartDashboard.getNumber("ELEVATOR_KD", m_kd);

            var apply = false;
            if (kG != m_kg) {
                m_kg = kG;
                apply = true;
            }
            if (kV != m_kv) {
                m_kv = kV;
                apply = true;
            }
            if (kA != m_ka) {
                m_ka = kA;
                apply = true;
            }

            if (kS != m_ks) {
                m_ks = kS;
                apply = true;
            }

            if (kP != m_kp) {
                m_kp = kP;
                apply = true;
            }
            if (kI != m_ki) {
                m_ki = kI;
                apply = true;
            }
            if (kD != m_kd) {
                m_kd = kD;
                apply = true;
            }

            if (apply) {
                var config = new TalonFXConfiguration();
                var leader_configurator = m_leaderMotor.getConfigurator();
                var follower_configurator = m_followerMotor.getConfigurator();

                leader_configurator.refresh(config);
                config.Slot0.kG = kG;
                config.Slot0.kV = kV;
                config.Slot0.kA = kA;

                config.Slot0.kS = kS;

                config.Slot0.kP = kP;
                config.Slot0.kI = kI;
                config.Slot0.kD = kD;

                leader_configurator.apply(config);
                follower_configurator.apply(config);
            }
        }

        // looks ugly, but compiler optimizes nicely
        if (RobotState.ENABLE_AUTOMATIC_ELEVATOR_CONTROL) {
            if (m_desiredControlType == DesiredControlType.AUTOMATIC)
                handleAutomatic();
            else
                handleManual();
        } else
            handleManual();

        m_manualPositionPublisher.set(m_manualPosition);
        m_desiredControlTypePublisher.set(m_desiredControlType.toString());

        m_leaderMotorPosition.refresh();
        m_followerMotorPosition.refresh();
        m_leaderMotorVelocity.refresh();

        m_PIDPositionReference.refresh();
        m_PIDPositionSlope.refresh();

        final double leaderMotorPosition = m_leaderMotorPosition.getValueAsDouble();
        m_followerMotorPositionPublisher.set(m_followerMotorPosition.getValueAsDouble());
        m_leaderMotorPositionPublisher.set(leaderMotorPosition);
        m_leaderMotorVelocityPublisher.set(m_leaderMotorVelocity.getValueAsDouble());

        final double PIDPositionReference = m_PIDPositionReference.getValueAsDouble();
        m_PIDPositionReferencePublisher.set(PIDPositionReference);
        m_PIDPositionSlopePublisher.set(m_PIDPositionSlope.getValueAsDouble());

        m_statePublisher.set(m_state.toString());
        m_desiredStatePublisher.set(m_state.toString());
        m_positionPublisher.set(m_position.toString());
        m_manualDirectionPublisher.set(m_manualDirection.toString());

        m_isAtTargetPositionPublisher.set(getIsAtTargetPosition());

        final double mechPositionToUse = Robot.isSimulation() ? PIDPositionReference : leaderMotorPosition;
        ElevatorMechanisms.ligament.setLength(ElevatorMechanisms.baseHeightMeters + Units.inchesToMeters(
            ElevatorMechanisms.maxHeightInches * (mechPositionToUse / ElevatorConstants.MAX_POSITION_ROTATIONS)
        ));
    }

    private void handleAutomatic() {
        // TODO: verify this should be IDLE
        // m_state = RobotState.getElevatorHasClearance() ? m_desiredState : ElevatorState.IDLE;
        m_state = m_desiredState;

        switch (m_state) {
            case HOME:
                setAutomaticPosition(ElevatorPosition.HOME);
                break;
            case IDLE:
                setAutomaticPosition(ElevatorPosition.IDLE);
                break;
            case TARGET:
                switch (RobotState.getTargetScorePosition()) {
                    case NONE:
                        setAutomaticPosition(ElevatorPosition.IDLE);
                        break;

                    case CORAL_STATION:
                        setAutomaticPosition(ElevatorPosition.CORAL_STATION);
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

        m_leaderMotor.setControl(desiredControl);
    }
    private void handleManual() {
        final double increment = 0.05;

        switch (m_manualDirection) {
            case NONE:
                break;
            case UP:
                if (RobotState.getElevatorHasClearance())
                    incrementManualPosition(increment);
                break;
            case DOWN:
                incrementManualPosition(-increment);
                break;
        }

        m_leaderMotor.setControl(m_positionControl.withPosition(m_manualPosition));
        // m_leaderMotor.setControl(m_brake);
    }

    public class TempGoUntilTargetIncreaseCommand extends Command {
        private double m_targetIncreaseInPosition;
        private double m_initialPosition;

        TempGoUntilTargetIncreaseCommand(double targetIncreaseInPosition) {
            m_targetIncreaseInPosition = targetIncreaseInPosition;
        }

        @Override
        public void initialize() {
            m_initialPosition = m_leaderMotorPosition.refresh().getValueAsDouble();
        }

        @Override
        public void execute() {
            m_leaderMotor.setControl(new DutyCycleOut(0.04));
        }

        @Override
        public boolean isFinished() {
            var position = m_leaderMotorPosition.refresh().getValueAsDouble();
            return position >= (m_initialPosition + m_targetIncreaseInPosition);
        }

        @Override
        public void end(boolean isInterrupted) {
            m_leaderMotor.setControl(m_brake);
        }
    }
    @Override
    public Command getTempGoUntilTargetIncreaseCommand(double targetIncreaseInPosition) {
        return new TempGoUntilTargetIncreaseCommand(targetIncreaseInPosition);
    }

    @Override
    public Command getTempHoldPositionCommand() {
        return run(() -> {
            m_leaderMotor.setControl(new DutyCycleOut(m_kg / 12d));
        });
    }

    public class TimeTravelCommand extends Command {
        private final double m_targetPosition;
        private Timer m_timer = new Timer();

        public TimeTravelCommand(double targetPosition) {
            m_targetPosition = targetPosition;
        }

        @Override
        public void initialize() {
            m_timer.start();
        }

        @Override
        public void execute() {
            m_leaderMotor.setControl(m_positionControl.withPosition(m_targetPosition));
        }

        @Override
        public boolean isFinished() {
            // FIXME: try this with getIsAtTarget
            var position = m_leaderMotorPosition.refresh().getValueAsDouble();
            return position >= m_targetPosition;
        }

        @Override
        public void end(boolean isInterrupted) {
            m_timer.stop();
            m_leaderMotor.setControl(m_brake);
            SmartDashboard.putNumber("ELEVATOR_TIME_TAKEN", m_timer.get());
        }
    }
    @Override
    public Command getTimeTravelCommand(double targetPosition) {
        return new TimeTravelCommand(targetPosition);
    }
}
