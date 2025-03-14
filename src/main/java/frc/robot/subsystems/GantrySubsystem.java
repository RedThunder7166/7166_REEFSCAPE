// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GantryConstants;
import frc.robot.Constants.IntakeOuttakeConstants;
import frc.robot.OurUtils.DIOInterface;
import frc.robot.OurUtils;
import frc.robot.Robot;
import frc.robot.RobotState;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.RobotState.IntakeState;
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
    private final LaserCan m_gantryLaser = new LaserCan(GantryConstants.GANTRY_LASER_ID);

    // private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final DutyCycleOut m_intakeDutyCycleOut = new DutyCycleOut(0);
    private final DutyCycleOut m_gantryDutyCycleOut = new DutyCycleOut(0);
    // private final VoltageOut m_voltageOut = new VoltageOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    private static final boolean tuneWithNetworkTables = true;

    private static final double m_pidControllerP = 0.0015; // 0.002; 0.00149; 0.00225; 0.0016
    private static final double m_pidControllerI = 0;
    private static final double m_pidControllerD = 0;

    private static final double m_feedForwardS = 0.03;
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
            5000)
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

    private Measurement m_gantryLaserMeasurement;

    public final DIOInterface m_elevatorClearanceSensor = OurUtils.getDIO(GantryConstants.ELEVATOR_CLEARANCE_SENSOR_ID);
    public final DIOInterface m_scoreEnterSensor = OurUtils.getDIO(GantryConstants.SCORE_ENTER_SENSOR_ID);
    public final DIOInterface m_scoreExitSensor = OurUtils.getDIO(GantryConstants.SCORE_EXIT_SENSOR_ID);

    public boolean m_elevatorClearanceSensorTripped = false;
    public boolean m_scoreEnterSensorTripped = false;
    public boolean m_scoreExitSensorTripped = false;

    private final DoublePublisher m_gantryMotorPositionPublisher = RobotState.robotStateTable.getDoubleTopic("GantryMotorPosition").publish();
    private final DoublePublisher m_gantryPositionMetersPublisher = RobotState.robotStateTable.getDoubleTopic("GantryPositionMeters").publish();
    private final DoublePublisher m_PIDPositionReferencePublisher = RobotState.robotStateTable.getDoubleTopic("GantryPIDPositionReferencePosition").publish();

    private final StringPublisher m_gantryLaserMeasurementPublisher = RobotState.robotStateTable.getStringTopic("GantryLaserMeasurement").publish();
    // private final StringPublisher m_gantryLaserAmbientPublisher = RobotState.robotStateTable.getStringTopic("GantryLaserAmbient").publish();

    private final BooleanPublisher m_elevatorClearanceSensorPublisher = RobotState.robotStateTable.getBooleanTopic("GantryElevatorClearanceSensor").publish();
    private final BooleanPublisher m_scoreEnterSensorPublisher = RobotState.robotStateTable.getBooleanTopic("GantryScoreEnterSensor").publish();
    private final BooleanPublisher m_scoreExitSensorPublisher = RobotState.robotStateTable.getBooleanTopic("GantryScoreExitSensor").publish();

    public GantrySubsystem() {
        // FIXME: tune gantry PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.kP = 20;

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

        try {
            m_gantryLaser.setRangingMode(LaserCan.RangingMode.SHORT);
            m_gantryLaser.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 4, 4));
            m_gantryLaser.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_50MS);
        } catch (ConfigurationFailedException e) {
            DriverStation.reportWarning("Could not apply configs to LaserCAN " + GantryConstants.GANTRY_LASER_ID + "; error code: " + e.toString(), false);
        }

        resetPositionStuff();

        RobotState.addTelemetry(() -> {
            m_gantryMotorPosition.refresh();
            m_gantryMotorVelocity.refresh();
            m_gantryMotorAcceleration.refresh();
            m_PIDPositionReference.refresh();
            // BaseStatusSignal.waitForAll(0.020, m_gantryMotorPosition, m_gantryMotorVelocity, m_gantryMotorAcceleration, m_PIDPositionReference);

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

            var measurement = m_gantryLaserMeasurement;
            if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
                m_gantryLaserMeasurementPublisher.set(measurement.distance_mm + "mm");
                // m_gantryLaserAmbientPublisher.set(measurement.ambient + "");
            } else {
                m_gantryLaserMeasurementPublisher.set("INVALID");
                // m_gantryLaserAmbientPublisher.set("INVALID");
            }

            final double mechPositionToUse = Robot.isSimulation() ? PIDPositionReference : gantryMotorPosition;
            final boolean mechPositionIsPositive = mechPositionToUse > 0;
            final double mechPositionSign = mechPositionIsPositive ? 1 : -1;
            double length = GantryMechanisms.widthInches * (mechPositionIsPositive ? (mechPositionToUse / GantryConstants.MAX_POSITION_ROTATIONS) : (mechPositionToUse / GantryConstants.MIN_POSITION_ROTATIONS));
            length = Units.inchesToMeters(length * mechPositionSign);
            if (Math.abs(length) == 0)
                length = mechPositionSign * 0.05;
            GantryMechanisms.ligament.setLength(length);
        });
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
    public boolean getIsAtTargetPosition() {
        // return true if we're not actively targeting a position / we're simulating
        if ((m_desiredControlType == DesiredControlType.AUTOMATIC && m_position == GantryPosition.IDLE) || Robot.isSimulation())
            return true;

        // double err = Math.abs(m_PIDPositionError.refresh().getValueAsDouble());
        // SmartDashboard.putNumber("GantryPIDError", err);
        // return err <= GantryConstants.POSITION_ERROR_THRESHOLD;
        double err = Math.abs(m_pidController.getPositionError());
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
            // switch (m_position) {
            //     case IDLE:
            //         break;
            //     default:
            //         setManualPosition(GantryConstants.millimetersToEncoderUnits(m_position.m_position));
            //         break;
            // }
            setDesiredControlType(DesiredControlType.MANUAL);
            setIdle();
            setManualDirection(desiredDirection);
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
        m_elevatorClearanceSensorTripped = m_elevatorClearanceSensor.get();
        m_scoreEnterSensorTripped = m_scoreEnterSensor.get();
        m_scoreExitSensorTripped = m_scoreExitSensor.get();

        m_gantryLaserMeasurement = m_gantryLaser.getMeasurement();

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

    private double m_feedForwardLastSpeed = 0;
    private double m_feedForwardLastTime = Timer.getFPGATimestamp();
    private boolean trySetPositionMM(double position) {
        var laserMeasurement = m_gantryLaserMeasurement;
        if (laserMeasurement == null)
            return false;

        m_automaticPositionRotationsPublisher.set(position); // FIXME: this is not rotations!!!
        var output = m_pidController.calculate(laserMeasurement.distance_mm, position);
        var velocity = m_pidController.getSetpoint().velocity;
        var timeStamp = Timer.getFPGATimestamp();

        double acceleration = (velocity - m_feedForwardLastSpeed) / (timeStamp - m_feedForwardLastTime);
        output += m_feedForward.calculate(velocity, acceleration);

        output = -output;
        SmartDashboard.putNumber("GANTRY_PID_OUTPUT", output);
        m_gantryMotor.setControl(m_gantryDutyCycleOut.withOutput(output));

        m_feedForwardLastSpeed = velocity;
        m_feedForwardLastTime = timeStamp;

        return true;
    }

    private void handleAutomatic() {
        // TODO: maybe if elevator has no clearance and state is score, set position to coralstation?
        switch (m_state) {
            case IDLE:
                setAutomaticPosition(GantryPosition.IDLE);
                break;
            case TARGET:
                switch (RobotState.getTargetScorePosition()) {
                    case NONE:
                        setAutomaticPosition(GantryPosition.IDLE);
                        break;

                    case CORAL_STATION:
                        setAutomaticPosition(GantryPosition.CORAL_STATION);
                        break;

                    case L1:
                        setAutomaticPosition(GantryPosition.TROUGH);
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
        }

        ControlRequest desiredControl = m_brake;

        Optional<Double> targetPositionMillimeters = Optional.empty();

        switch (m_position) {
            case IDLE:
                desiredControl = m_brake; // redundant?
                break;
            case CORAL_STATION:
            case TROUGH:
                // desiredControl = m_positionControl.withPosition(m_position.m_position);
                targetPositionMillimeters  = Optional.of(m_position.m_position);
                break;
            default:
                double position = m_position.m_position;
                // var reefTargetHorizontalDistance = RobotState.getReefTargetHorizontalDistance();
                // if (reefTargetHorizontalDistance.isPresent()) {
                //     double distance = reefTargetHorizontalDistance.get();
                //     double desiredPosition = position - GantryConstants.metersToEncoderUnits(distance);
                //     if (desiredPosition >= GantryConstants.MIN_POSITION_ROTATIONS && desiredPosition <= GantryConstants.MAX_POSITION_ROTATIONS)
                //         position = desiredPosition;
                // }
                // desiredControl = m_positionControl.withPosition(position);

                var reefTargetHorizontalDistance = RobotState.getReefTargetHorizontalDistance();
                if (reefTargetHorizontalDistance.isPresent()) {
                    double distance = reefTargetHorizontalDistance.get();
                    distance *= 1000; // meters to mm
                    double desiredPosition = position + distance;
                    if (desiredPosition >= GantryConstants.MIN_POSITION_MM && desiredPosition <= GantryConstants.MAX_POSITION_MM)
                        position = desiredPosition;
                }

                targetPositionMillimeters  = Optional.of(position);
                break;
        }

        // if (desiredControl == m_positionControl)
        //     m_automaticPositionRotationsPublisher.set(m_positionControl.Position);

        // m_gantryMotor.setControl(desiredControl);

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
        // TODO: score motor should not be controlled purely off intake
        ControlRequest targetRequest = m_brake;

        // switch (RobotState.getIntakeState()) {
        //     case IDLE:
        //         targetRequest = m_brake; // redundant?
        //         break;
        //     case OUT:
        //         targetRequest = m_dutyCycleOut.withOutput(IntakeOuttakeConstants.BACKWARD_OUTPUT);
        //         break;
        //     case IN:
        //         targetRequest = m_dutyCycleOut.withOutput(IntakeOuttakeConstants.FORWARD_OUTPUT);
        //         break;
        // }

        SmartDashboard.putBoolean("CORALISGOOD", false);

        RobotState.IntakeState intakeState = RobotState.getIntakeState();
        if (intakeState == IntakeState.OUT) {
            targetRequest = m_intakeDutyCycleOut.withOutput(IntakeOuttakeConstants.BACKWARD_OUTPUT);
        } else {
            final boolean isGoodToMove = !RobotState.getWantsToScore() && (m_desiredControlType == DesiredControlType.MANUAL || m_position != GantryPosition.TROUGH);
            // TargetScorePosition targetScorePosition = RobotState.getTargetScorePosition();

            if (m_elevatorClearanceSensorTripped)
                // if we are in automatic control and we are targeting a non-coral-station position, don't move motor
                // if (m_desiredControlType == DesiredControlType.MANUAL || (m_position == GantryPosition.CORAL_STATION || m_position == GantryPosition.IDLE))
                if (isGoodToMove)
                    targetRequest = m_intakeDutyCycleOut.withOutput(IntakeOuttakeConstants.CRAWL_FORWARD_OUTPUT);
                    // targetRequest = m_voltageOut.withOutput(IntakeOuttakeConstants.CRAWL_FORWARD_VOLTAGE);
                else
                    targetRequest = m_brake;
            // else if (!m_scoreEnterSensorTripped && (targetScorePosition == TargetScorePosition.NONE || targetScorePosition == TargetScorePosition.CORAL_STATION))
            else if (!m_scoreEnterSensorTripped && isGoodToMove)
                targetRequest = m_intakeDutyCycleOut.withOutput(IntakeOuttakeConstants.CRAWL_BACKWARD_OUTPUT);
            else
                targetRequest = m_brake;
        }

        boolean coralIsGood = m_scoreExitSensorTripped && m_scoreEnterSensorTripped && !m_elevatorClearanceSensorTripped;
        if (coralIsGood) {
            SmartDashboard.putBoolean("CORALISGOOD", true);
        }
        RobotState.setCoralIsGood(coralIsGood);

        // boolean elevatorHasClearance = !m_elevatorClearanceSensorTripped && (m_scoreEnterSensorTripped == m_scoreExitSensorTripped);
        boolean elevatorHasClearance = !m_elevatorClearanceSensorTripped;
        RobotState.setElevatorHasClearance(elevatorHasClearance);

        // FIXME: below has a removed check (was elevatorHasClearance WHICH WAS DUMB it should be just [exit and enter])
        if (m_scoreExitSensorTripped && RobotState.getWantsToScore())
            targetRequest = m_intakeDutyCycleOut.withOutput(IntakeOuttakeConstants.FORWARD_OUTPUT);

        m_scoreMotor.setControl(targetRequest);
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
