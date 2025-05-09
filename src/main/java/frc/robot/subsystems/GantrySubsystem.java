// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GantryConstants;
import frc.robot.OurUtils.DIOInterface;
import frc.robot.OurUtils;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.RobotState.IntakeState;
import frc.robot.RobotState.TargetScorePosition;
import frc.robot.subsystems.Mechanisms.GantryMechanisms;
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
        public void incrementManualPosition(double value) { }

        @Override
        public void resetManualPosition() { }

        @Override
        public void setDesiredControlType(DesiredControlType desiredControlType) { }

        @Override
        public void resetMotorPosition() { }

        @Override
        public void resetPositionStuff() { }

        @Override
        public boolean getIsTargetingAScoreLocation() {
            return true;
        }

        @Override
        public boolean getIsAtPosition(TargetScorePosition position) {
            return true;
        }

        @Override
        public boolean getIsAtTargetPosition() {
            return true;
        }

        @Override
        public boolean getScoreEnterSensorTripped() {
            return false;
        }
        @Override
        public boolean getScoreExitSensorTripped() {
            return false;
        }

        @Override
        public Command getManualLeftCommand() {
            return Commands.none();
        }
        @Override
        public Command getManualRightCommand() {
            return Commands.none();
        }

        @Override
        public void manualSeedThisPosition() { }

        @Override
        public Command getGantryResetPositionCommand() {
            return Commands.none();
        }
    }

    private static GantrySubsystemInterface singleton = null;

    public static synchronized GantrySubsystemInterface getSingleton() {
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
    public synchronized void setAutomaticState(GantryState desiredState) {
        m_state = desiredState;
    }
    private final StringPublisher m_statePublisher = RobotState.robotStateTable.getStringTopic("GantryState").publish();

    @Override
    public synchronized void setIdle() {
        setDesiredControlType(DesiredControlType.AUTOMATIC);
        setAutomaticState(GantryState.IDLE);
        m_state = GantryState.IDLE;
        m_position = GantryPosition.IDLE;
    }

    private static enum GantryPosition {
        // IDLE(GantryConstants.CORAL_STATION_POSITION_ROTATIONS), // m_position here should never be used, so we have it coral station to be safe
        // CORAL_STATION(GantryConstants.CORAL_STATION_POSITION_ROTATIONS),
        // REEF_LEFT(GantryConstants.REEF_LEFT_POSITION_ROTATIONS),
        // REEF_RIGHT(GantryConstants.REEF_RIGHT_POSITION_ROTATIONS)

        IDLE(GantryConstants.CORAL_STATION_POSITION_MM), // m_position here should never be used, so we have it coral station to be safe
        CORAL_STATION(GantryConstants.CORAL_STATION_POSITION_MM),
        TROUGH(GantryConstants.CORAL_STATION_POSITION_MM),
        REEF_LEFT(GantryConstants.REEF_LEFT_POSITION_MM),
        REEF_RIGHT(GantryConstants.REEF_RIGHT_POSITION_MM)

        ;
        private final double m_position;

        GantryPosition(double position) {
            m_position = position;
        }
    }
    private GantryPosition m_position = GantryPosition.IDLE;
    private synchronized void setAutomaticPosition(GantryPosition desiredPosition) {
        m_position = desiredPosition;
    }
    private final StringPublisher m_positionPublisher = RobotState.robotStateTable.getStringTopic("GantryPosition").publish();
    private final DoublePublisher m_automaticPositionRotationsPublisher = RobotState.robotStateTable.getDoubleTopic("GantryAutomaticPositionRotations").publish();

    private GantryManualDirection m_manualDirection = GantryManualDirection.NONE;
    private GantryManualDirection m_oldManualDirection = m_manualDirection;
    public synchronized void setManualDirection(GantryManualDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }
    private final StringPublisher m_manualDirectionPublisher = RobotState.robotStateTable.getStringTopic("GantryManualDirection").publish();

    private double m_manualPosition = 0;
    private synchronized void setManualPosition(double newValue) {
        // if (newValue < GantryConstants.MIN_POSITION_ROTATIONS)
        //     newValue = GantryConstants.MIN_POSITION_ROTATIONS;
        // if (newValue > GantryConstants.MAX_POSITION_ROTATIONS)
        //     newValue = GantryConstants.MAX_POSITION_ROTATIONS;

        if (newValue < GantryConstants.MIN_POSITION_MM)
            newValue = GantryConstants.MIN_POSITION_MM;
        if (newValue > GantryConstants.MAX_POSITION_MM)
            newValue = GantryConstants.MAX_POSITION_MM;

        m_manualPosition = newValue;
    }
    @Override
    public synchronized void incrementManualPosition(double value) {
        setManualPosition(m_manualPosition + value);
    }
    @Override
    public synchronized void resetManualPosition() {
        setManualPosition(0);
    }
    private final DoublePublisher m_manualPositionPublisher = RobotState.robotStateTable.getDoubleTopic("GantryManualTargetPosition").publish();

    private DesiredControlType m_desiredControlType = DesiredControlType.AUTOMATIC;
    @Override
    public synchronized void setDesiredControlType(DesiredControlType desiredControlType) {
        m_desiredControlType = desiredControlType;
    }
    private final StringPublisher m_desiredControlTypePublisher = RobotState.robotStateTable.getStringTopic("GantryDesiredControlType").publish();

    private boolean m_isResettingPosition = false;
    private boolean m_oldIsResettingPosition = m_isResettingPosition;

    private final TalonFX m_gantryMotor = new TalonFX(GantryConstants.GANTRY_MOTOR_ID);
    private final TalonFX m_scoreMotor = new TalonFX(GantryConstants.SCORE_MOTOR_ID);

    private static class GenericDistanceSensor {
        public static enum SensorType {
            LASERCAN,
            CANRANGE
        };
        public final SensorType m_type;

        private LaserCan m_laserCan = null;
        private Measurement m_laserCanMeasurement = null;

        private CANrange m_canRange = null;
        private StatusSignal<Distance> m_canRangeDistance = null;
        private StatusSignal<MeasurementHealthValue> m_canRangeHealth = null;

        public GenericDistanceSensor(SensorType type) {
            m_type = type;
            switch (type) {
                case LASERCAN:
                    m_laserCan = new LaserCan(GantryConstants.GANTRY_LASER_ID);
                    try {
                        m_laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
                        m_laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
                        m_laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
                    } catch (ConfigurationFailedException e) {
                        DriverStation.reportWarning("Could not apply configs to LaserCAN " + GantryConstants.GANTRY_LASER_ID + "; error code: " + e.toString(), false);
                    }
                    m_laserCanMeasurement = m_laserCan.getMeasurement();
                    break;
                case CANRANGE:
                    m_canRange = new CANrange(GantryConstants.GANTRY_CANRANGE_ID);

                    CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();

                    canRangeConfig.FovParams.FOVCenterX = 0.75;
                    canRangeConfig.FovParams.FOVCenterY = 2.5;
                    canRangeConfig.FovParams.FOVRangeX = 6.75;
                    canRangeConfig.FovParams.FOVRangeY = 6.75;

                    OurUtils.tryApplyConfig(m_canRange, canRangeConfig);

                    m_canRangeDistance = m_canRange.getDistance();
                    m_canRangeHealth = m_canRange.getMeasurementHealth();
                    break;
            }
        }

        public void update() {
            switch (m_type) {
                case LASERCAN:
                    m_laserCanMeasurement = m_laserCan.getMeasurement();
                    break;
                case CANRANGE:
                    m_canRangeDistance.refresh();
                    m_canRangeHealth.refresh();
                    break;
            }
        }

        Optional<Double> getDistance() {
            switch (m_type) {
                case LASERCAN:
                    final Measurement measurement = m_laserCanMeasurement;
                    if (measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
                        return Optional.empty();

                    return Optional.of((double) measurement.distance_mm);
                case CANRANGE:
                    if (m_canRangeHealth.getValue() != MeasurementHealthValue.Good)
                        return Optional.empty();

                    return Optional.of(m_canRangeDistance.getValueAsDouble() * 1000d);
            }
            return Optional.empty();
        }
    }

    private static final GenericDistanceSensor m_distanceSensor = new GenericDistanceSensor(GenericDistanceSensor.SensorType.CANRANGE);

    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut m_scoreDutyCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut m_gantryDutyCycleOut = new DutyCycleOut(0);
    // private final VoltageOut m_voltageOut = new VoltageOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    private static final boolean influenceMotorPositionFromDistanceSensor = false;
    private static final boolean tuneWithNetworkTables = true;

    private static final double m_pidControllerP = 0.0025; // 0.002; 0.00149; 0.00225; 0.0016; 0.0015; 0.0019; 0.00165; 0.0011
    private static final double m_pidControllerI = 0; // 0; 0.0004
    private static final double m_pidControllerD = 0; // 0.00001; 0.000125

    private static final double m_feedForwardS = 0.029; // 0.03; 0.0253
    private static final double m_feedForwardV = 0; // 0.0001
    { 
        if (tuneWithNetworkTables) {
            SmartDashboard.putNumber("GANTRY_P", m_pidControllerP);
            SmartDashboard.putNumber("GANTRY_I", m_pidControllerI);
            SmartDashboard.putNumber("GANTRY_D", m_pidControllerD);
            SmartDashboard.putNumber("GANTRY_S", m_feedForwardS);
            SmartDashboard.putNumber("GANTRY_V", m_feedForwardV);
        } 
    }
    private final ProfiledPIDController m_pidController = new ProfiledPIDController(
        m_pidControllerP,
        m_pidControllerI,
        m_pidControllerD,
        new TrapezoidProfile.Constraints(
            5000,
            10000) // 5000
    );
    // private final PIDController m_pidController = new PIDController(
    //     m_pidControllerP,
    //     m_pidControllerI,
    //     m_pidControllerD
    // );
    private SimpleMotorFeedforward m_feedForward = new SimpleMotorFeedforward(
        m_feedForwardS,
        m_feedForwardV
    );

    private final StatusSignal<Angle> m_gantryMotorPosition = m_gantryMotor.getPosition();
    private final StatusSignal<AngularVelocity> m_gantryMotorVelocity = m_gantryMotor.getVelocity();
    private final StatusSignal<AngularAcceleration> m_gantryMotorAcceleration = m_gantryMotor.getAcceleration();

    private final StatusSignal<Double> m_PIDPositionReference = m_gantryMotor.getClosedLoopReference();
    // private final StatusSignal<Double> m_PIDPositionError = m_gantryMotor.getClosedLoopError();

    private double m_gantryTargetPositionRotations = 0;

    public final DIOInterface m_elevatorClearanceSensor = OurUtils.getDIO(GantryConstants.ELEVATOR_CLEARANCE_SENSOR_ID);
    public final DIOInterface m_scoreEnterSensor = OurUtils.getDIO(GantryConstants.SCORE_ENTER_SENSOR_ID);
    public final DIOInterface m_scoreExitSensor = OurUtils.getDIO(GantryConstants.SCORE_EXIT_SENSOR_ID);

    public boolean m_elevatorClearanceSensorTripped = false;
    public boolean m_scoreEnterSensorTripped = false;
    public boolean m_scoreExitSensorTripped = false;

    private final DoublePublisher m_gantryMotorPositionPublisher = RobotState.robotStateTable.getDoubleTopic("GantryMotorPosition").publish();
    private final DoublePublisher m_gantryPositionMetersPublisher = RobotState.robotStateTable.getDoubleTopic("GantryPositionMeters").publish();
    private final DoublePublisher m_PIDPositionReferencePublisher = RobotState.robotStateTable.getDoubleTopic("GantryPIDPositionReferencePosition").publish();

    private final StringPublisher m_gantryDistanceMeasurementPublisher = RobotState.robotStateTable.getStringTopic("GantryDistanceMeasurement").publish();

    private final BooleanPublisher m_elevatorClearanceSensorPublisher = RobotState.robotStateTable.getBooleanTopic("GantryElevatorClearanceSensor").publish();
    private final BooleanPublisher m_scoreEnterSensorPublisher = RobotState.robotStateTable.getBooleanTopic("GantryScoreEnterSensor").publish();
    private final BooleanPublisher m_scoreExitSensorPublisher = RobotState.robotStateTable.getBooleanTopic("GantryScoreExitSensor").publish();

    private double m_lastInfluencedDistance = 0;
    // private double m_lastInfluencedTimestamp = Timer.getFPGATimestamp();

    private void tryInfluencePosition(double distance) {
        if (!DriverStation.isEnabled())
            return;
        final double distance_difference = Math.abs(m_lastInfluencedDistance - distance);
        if (distance_difference < 5)
            return;

        // don't set config multiple times per second
        // final double timestamp = Timer.getFPGATimestamp();
        // if (Math.abs(m_lastInfluencedTimestamp - timestamp) < 0.5)
        //     return;

        m_lastInfluencedDistance = distance;
        // m_lastInfluencedTimestamp = timestamp;
        m_gantryMotor.setPosition(GantryConstants.millimetersToEncoderUnits(distance), 0.05);
    }

    public GantrySubsystem() {
        // FIXME: tune gantry PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 3.2; // 20; 5.3; 4.75
        motorConfig.Slot0.kS = 0.029; // 0.27; 0.025
        motorConfig.Slot0.kD = 0.2;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        motorConfig.CurrentLimits.StatorCurrentLimit = 20;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 30;

        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = GantryConstants.MAX_POSITION_ROTATIONS;
        // motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = GantryConstants.MIN_POSITION_ROTATIONS;

        var motionMagicConfigs = motorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 40;
        motionMagicConfigs.MotionMagicAcceleration = 120;

        motorConfig.Feedback.SensorToMechanismRatio = 54d / 12d;

        OurUtils.tryApplyConfig(m_gantryMotor, motorConfig);

        TalonFXConfiguration scoreMotorConfig = new TalonFXConfiguration();
        scoreMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        OurUtils.tryApplyConfig(m_scoreMotor, scoreMotorConfig);

        resetPositionStuff();

        RobotState.addTelemetry(() -> {
            m_gantryMotorPosition.refresh();
            m_gantryMotorVelocity.refresh();
            m_gantryMotorAcceleration.refresh();
            m_PIDPositionReference.refresh();
            // BaseStatusSignal.waitForAll(0.020, m_gantryMotorPosition, m_gantryMotorVelocity, m_gantryMotorAcceleration, m_PIDPositionReference);

            m_distanceSensor.update();
            m_distanceSensor.getDistance().ifPresentOrElse(
                (Double value) -> m_gantryDistanceMeasurementPublisher.set("" + value.doubleValue()),
                () -> m_gantryDistanceMeasurementPublisher.set("INVALID")
            );

            final double gantryMotorPosition = m_gantryMotorPosition.getValueAsDouble();
            final double gantryPositionMeters = GantryConstants.encoderUnitsToMeters(gantryMotorPosition);
            m_gantryMotorPositionPublisher.set(gantryMotorPosition);
            m_gantryPositionMetersPublisher.set(gantryPositionMeters);

            final double PIDPositionReference = m_PIDPositionReference.getValueAsDouble();
            m_PIDPositionReferencePublisher.set(PIDPositionReference);

            m_statePublisher.set(m_state.toString());
            m_positionPublisher.set(m_position.toString());
            m_manualDirectionPublisher.set(m_manualDirection.toString());

            m_isAtTargetPositionPublisher.set(getIsAtTargetPosition());

            m_elevatorClearanceSensorPublisher.set(m_elevatorClearanceSensorTripped);
            m_scoreEnterSensorPublisher.set(m_scoreEnterSensorTripped);
            m_scoreExitSensorPublisher.set(m_scoreExitSensorTripped);

            m_manualPositionPublisher.set(m_manualPosition);
            m_desiredControlTypePublisher.set(m_desiredControlType.toString());

            if (tuneWithNetworkTables) {
                m_pidController.setP(SmartDashboard.getNumber("GANTRY_P", m_pidControllerP));
                m_pidController.setI(SmartDashboard.getNumber("GANTRY_I", m_pidControllerI));
                m_pidController.setD(SmartDashboard.getNumber("GANTRY_D", m_pidControllerD));
                m_feedForward = new SimpleMotorFeedforward(
                    SmartDashboard.getNumber("GANTRY_S", m_feedForwardS),
                    SmartDashboard.getNumber("GANTRY_V", m_feedForwardV)
                );
            }

            final double mechPositionToUse = Robot.isSimulation() ? PIDPositionReference : gantryMotorPosition;
            final boolean mechPositionIsPositive = mechPositionToUse > 0;
            final double mechPositionSign = mechPositionIsPositive ? 1 : -1;
            double length = GantryMechanisms.widthInches * (mechPositionIsPositive ? (mechPositionToUse / GantryConstants.MAX_POSITION_ROTATIONS) : (mechPositionToUse / GantryConstants.MIN_POSITION_ROTATIONS));
            length = Units.inchesToMeters(length * mechPositionSign);
            if (Math.abs(length) == 0)
                length = mechPositionSign * 0.05d;
            GantryMechanisms.ligament.setLength(length);
        });

        if (influenceMotorPositionFromDistanceSensor)
            RobotState.addTelemetry(() -> {
                m_distanceSensor.getDistance().ifPresent(
                    (Double value) -> tryInfluencePosition(value)
                );
            }, 20);
    }

    @Override
    public void resetMotorPosition() {
        m_gantryMotor.setPosition(0);
    }

    @Override
    public void resetPositionStuff() {
        resetManualPosition();
        resetMotorPosition();
    }

    @Override
    public boolean getIsTargetingAScoreLocation() {
        if (m_desiredControlType == DesiredControlType.MANUAL)
            return false;

        if (m_state != GantryState.TARGET)
            return false;

        return true;
    }

    private boolean skipIsAtPositionCheck() {
        // we're not actively targeting a position or we're simulating
        return (m_desiredControlType == DesiredControlType.AUTOMATIC && m_position == GantryPosition.IDLE) || Robot.isSimulation();
    }

    private GantryPosition getGantryPositionFromTargetScorePosition(TargetScorePosition position) {
        switch (position) {
            case NONE:
                return GantryPosition.IDLE;

            case CORAL_STATION:
                return GantryPosition.CORAL_STATION;

            case L1:
                return GantryPosition.TROUGH;

            case L2_L:
            case L3_L:
            case L4_L:
                return GantryPosition.REEF_LEFT;

            case L2_R:
            case L3_R:
            case L4_R:
                return GantryPosition.REEF_RIGHT;
        }
        return null; // java is dumb; the above switch statement covers all cases... unless position is null
    }

    @Override
    public boolean getIsAtPosition(TargetScorePosition position) {
        if (skipIsAtPositionCheck())
            return true;

        final Optional<Double> distance = m_distanceSensor.getDistance();
        if (distance.isEmpty())
            return false;

        final double err = Math.abs(distance.get() - getGantryPositionFromTargetScorePosition(position).m_position);
        return err <= GantryConstants.POSITION_ERROR_THRESHOLD_MM;
    }

    @Override
    public boolean getIsAtTargetPosition() {
        if (skipIsAtPositionCheck())
            return true;


        final double err = influenceMotorPositionFromDistanceSensor ? GantryConstants.encoderUnitsToMillieters(Math.abs(m_gantryMotorPosition.getValueAsDouble() - m_gantryTargetPositionRotations))
            : Math.abs(m_pidController.getPositionError());

        SmartDashboard.putNumber("GantryPIDError", err);

        return err <= GantryConstants.POSITION_ERROR_THRESHOLD_MM;
    }
    private final BooleanPublisher m_isAtTargetPositionPublisher = RobotState.robotStateTable.getBooleanTopic("GantryIsAtTargetPosition").publish();

    @Override
    public synchronized boolean getScoreEnterSensorTripped() {
        return m_scoreEnterSensorTripped;
    }

    @Override
    public synchronized boolean getScoreExitSensorTripped() {
        return m_scoreExitSensorTripped;
    }

    private Command makeManualCommand(GantryManualDirection desiredDirection) {
        return startEnd(() -> {
            switch (m_position) {
                case IDLE:
                    break;
                default:
                    setManualPosition(m_position.m_position);
            }
            setIdle();
            setManualDirection(desiredDirection);
            setDesiredControlType(DesiredControlType.MANUAL);
        }, () -> {
            setManualDirection(GantryManualDirection.NONE);
        });
    }
    private final Command m_manualLeftCommand = makeManualCommand(GantryManualDirection.LEFT);
    private final Command m_manualRightCommand = makeManualCommand(GantryManualDirection.RIGHT);

    @Override
    public synchronized Command getManualLeftCommand() {
        return m_manualLeftCommand;
    }
    @Override
    public synchronized Command getManualRightCommand() {
        return m_manualRightCommand;
    }

    @Override
    public void periodic() {
        RobotState.setIsGantryAutoAdjustOutOfBounds(false);
        m_elevatorClearanceSensorTripped = m_elevatorClearanceSensor.get();
        m_scoreEnterSensorTripped = m_scoreEnterSensor.get();
        m_scoreExitSensorTripped = m_scoreExitSensor.get();

        if (m_isResettingPosition && !m_oldIsResettingPosition) {
            m_oldManualDirection = m_manualDirection;
            m_manualDirection = GantryManualDirection.LEFT;

            handleManual();
        } else 
        // looks ugly, but compiler optimizes nicely
        if (RobotState.ENABLE_AUTOMATIC_GANTRY_CONTROL) {
            if (m_desiredControlType == DesiredControlType.AUTOMATIC)
                handleAutomatic();
            else
                handleManual();
        } else
            handleManual();

        handleScoreMotor();
        m_oldIsResettingPosition = m_isResettingPosition;
    }

    private boolean trySetPositionMM(double position) {
        m_gantryTargetPositionRotations = GantryConstants.millimetersToEncoderUnits(position);

        final Optional<Double> distance = m_distanceSensor.getDistance();
        if (distance.isEmpty())
            return false;

        m_automaticPositionRotationsPublisher.set(position); // FIXME: change name; this is not rotations!!!
        if (influenceMotorPositionFromDistanceSensor) {
            m_gantryMotor.setControl(m_positionControl.withPosition(m_gantryTargetPositionRotations));
        } else {
            double output = m_pidController.calculate(distance.get(), position);
            output += m_feedForward.calculateWithVelocities(m_pidController.getSetpoint().velocity, m_pidController.getGoal().velocity);

            if (Math.abs(output) < 0.015)
                return true;

            SmartDashboard.putNumber("GANTRY_PID_OUTPUT", output);

            m_gantryMotor.setControl(m_gantryDutyCycleOut.withOutput(output));
        }

        return true;
    }

    private void handleAutomatic() {
        // TODO: maybe if elevator has no clearance and state is score, set position to coralstation?
        switch (m_state) {
            case IDLE:
                setAutomaticPosition(GantryPosition.IDLE);
                break;
            case TARGET:
                setAutomaticPosition(getGantryPositionFromTargetScorePosition(RobotState.getTargetScorePosition()));
                break;
        }

        ControlRequest desiredControl = m_brake;

        Optional<Double> targetPositionMillimeters = Optional.empty();

        switch (m_position) {
            case IDLE:
                desiredControl = m_brake; // redundant?
                break;
            case CORAL_STATION:
            case TROUGH:
                targetPositionMillimeters  = Optional.of(m_position.m_position);
                break;
            default:
                // auto adjust
                double position = m_position.m_position;

                var reefTargetHorizontalDistance = RobotState.getReefTargetHorizontalDistance();
                if (reefTargetHorizontalDistance.isPresent() && RobotState.getVisionPoseSuccess() && RobotState.getWeHaveCoral()) {
                    double distance = reefTargetHorizontalDistance.get();
                    distance *= 1000; // meters to mm
                    double desiredPosition = position + distance;
                    if (desiredPosition >= GantryConstants.MIN_POSITION_MM && desiredPosition <= GantryConstants.MAX_POSITION_MM)
                        position = desiredPosition;
                    else
                        RobotState.setIsGantryAutoAdjustOutOfBounds(true);
                }

                targetPositionMillimeters  = Optional.of(position);
                break;
        }

        if (!(targetPositionMillimeters.isPresent() && trySetPositionMM(targetPositionMillimeters.get())))
            m_gantryMotor.setControl(desiredControl);
    }
    private void handleManual() {
        final double increment = 10;

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

        trySetPositionMM(m_manualPosition);
    }

    private void handleScoreMotor() {
        ControlRequest targetRequest = m_brake;

        RobotState.IntakeState intakeState = RobotState.getIntakeState();
        if (intakeState == IntakeState.OUT) {
            targetRequest = m_scoreDutyCycleOut.withOutput(GantryConstants.BACKWARD_OUTPUT);
        // } else if (intakeState == IntakeState.NONE) {
        } else {
            // final boolean isGoodToMove = !RobotState.getWantsToScoreCoral() && (m_desiredControlType == DesiredControlType.MANUAL || m_position != GantryPosition.TROUGH);
            final boolean isGoodToMove = !RobotState.getWantsToScoreCoral();
            // TargetScorePosition targetScorePosition = RobotState.getTargetScorePosition();

            if (m_elevatorClearanceSensorTripped)
                if (isGoodToMove)
                    targetRequest = m_scoreDutyCycleOut.withOutput(GantryConstants.CRAWL_FORWARD_OUTPUT);
                else
                    targetRequest = m_brake;
            else if (!m_scoreEnterSensorTripped && (m_scoreExitSensorTripped || RobotState.getIntakeState() == IntakeState.IN) && isGoodToMove)
                targetRequest = m_scoreDutyCycleOut.withOutput(GantryConstants.CRAWL_BACKWARD_OUTPUT);
            else
                targetRequest = m_brake;
        }

        final boolean weHaveCoral = m_scoreExitSensorTripped && !m_elevatorClearanceSensorTripped;
        final boolean coralIsGood = weHaveCoral && m_scoreEnterSensorTripped;
        RobotState.setCoralIsGood(coralIsGood);
        RobotState.setWeHaveCoral(weHaveCoral);

        // boolean elevatorHasClearance = !m_elevatorClearanceSensorTripped && (m_scoreEnterSensorTripped == m_scoreExitSensorTripped);
        boolean elevatorHasClearance = !m_elevatorClearanceSensorTripped;
        RobotState.setElevatorHasClearance(elevatorHasClearance);

        // FIXME: below has a removed check (was elevatorHasClearance WHICH WAS DUMB it should be just [exit and enter])
        if (m_scoreExitSensorTripped && RobotState.getWantsToScoreCoral())
            targetRequest = m_scoreDutyCycleOut.withOutput(RobotState.getTargetScorePosition().getScoreOutput());

        m_scoreMotor.setControl(targetRequest);
    }

    public void manualSeedThisPosition() {
        if (influenceMotorPositionFromDistanceSensor)
            setManualPosition(GantryConstants.encoderUnitsToMillieters(m_gantryMotorPosition.getValueAsDouble()));
        else
            m_distanceSensor.getDistance().ifPresent((Double value) -> setManualPosition(value));
    }

    private final Command m_gantryResetPostionCommand = Commands.startEnd(() -> {
        m_desiredControlType = DesiredControlType.MANUAL;
        m_isResettingPosition = true;
    }, () -> {
        m_isResettingPosition = false;
        m_manualDirection = m_oldManualDirection;
        resetPositionStuff();
    }, this);

    @Override
    public Command getGantryResetPositionCommand() {
        return m_gantryResetPostionCommand;
    }
}
