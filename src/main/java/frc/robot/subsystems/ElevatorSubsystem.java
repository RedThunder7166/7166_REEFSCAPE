// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;

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
    private ElevatorState m_desiredState = ElevatorState.HOME;
    private final StringPublisher m_statePublisher = RobotState.m_robotStateTable.getStringTopic("ElevatorState").publish();
    private final StringPublisher m_desiredStatePublisher = RobotState.m_robotStateTable.getStringTopic("ElevatorDesiredState").publish();
    public void setAutomaticState(ElevatorState desiredState) {
        m_desiredState = desiredState;
    }

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
    private final StringPublisher m_positionPublisher = RobotState.m_robotStateTable.getStringTopic("ElevatorPosition").publish();
    private void setAutomaticPosition(ElevatorPosition desiredPosition) {
        m_position = desiredPosition;
    }
    private final DoublePublisher m_automaticPositionRotationsPublisher = RobotState.m_robotStateTable.getDoubleTopic("ElevatorAutomaticPositionRotations").publish();

    public static enum ElevatorManualDirection {
        NONE,
        UP,
        DOWN
    }
    private ElevatorManualDirection m_manualDirection = ElevatorManualDirection.NONE;
    private final StringPublisher m_manualDirectionPublisher = RobotState.m_robotStateTable.getStringTopic("ElevatorManualDirection").publish();
    public void setManualDirection(ElevatorManualDirection desiredManualDirection) {
        m_manualDirection = desiredManualDirection;
    }

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
    public void incrementManualPosition(double value) {
        setManualPosition(m_manualPosition + value);
    }
    public void resetManualPosition() {
        setManualPosition(0);
    }
    private final DoublePublisher m_manualPositionPublisher = RobotState.m_robotStateTable.getDoubleTopic("ElevatorManualTargetPosition").publish();

    private final TalonFX m_leaderMotor = new TalonFX(ElevatorConstants.LEADER_MOTOR_ID, Constants.CANIVORE_NAME);
    private final TalonFX m_followerMotor = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID, Constants.CANIVORE_NAME);

    private final MotionMagicVoltage m_positionControl = new MotionMagicVoltage(0).withSlot(0);
    private final VelocityVoltage m_velocityControl = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    private final StatusSignal<Angle> m_leaderMotorPosition = m_leaderMotor.getPosition();
    private final StatusSignal<Angle> m_followerMotorPosition = m_followerMotor.getPosition();

    private final DoublePublisher m_leaderMotorPositionPublisher = RobotState.m_robotStateTable.getDoubleTopic("ElevatorLeaderMotorPosition").publish();
    private final DoublePublisher m_followerMotorPositionPublisher = RobotState.m_robotStateTable.getDoubleTopic("ElevatorFollowerMotorPosition").publish();

    private DESIRED_CONTROL_TYPE m_desiredControlType = DESIRED_CONTROL_TYPE.AUTOMATIC;
    public void setDesiredControlType(DESIRED_CONTROL_TYPE desiredControlType) {
        m_desiredControlType = desiredControlType;
    }
    public DESIRED_CONTROL_TYPE getDesiredControlType() {
        return m_desiredControlType;
    }

    /* SOME NOTES
        minimum kG before elevator goes down: -0.175

        2.4 inches per rotation of the motor
     */
    private double m_kg = 0;
    private double m_kv = 4.89; // 1.6
    private double m_ka = 0;

    public ElevatorSubsystem() {
        // FIXME: tune elevator PID
        TalonFXConfiguration motorConfig = new TalonFXConfiguration();
        motorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        motorConfig.Slot0.kG = SmartDashboard.getNumber("ELEVATOR_KG", m_kg);
        motorConfig.Slot0.kV = SmartDashboard.getNumber("ELEVATOR_KV", m_kv);
        motorConfig.Slot0.kA = SmartDashboard.getNumber("ELEVATOR_KA", m_ka);
        // motorConfig.kP

        // TODO: once tuned, find peak voltage and set
        // motorConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
        //     .withPeakReverseVoltage(Volts.of(-8));

        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motorConfig.Feedback.SensorToMechanismRatio = 4.909;

        motorConfig.CurrentLimits.StatorCurrentLimit = 60;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 60;

        // FIXME: tune elevator Motion Magic; may be different per motor?
        // set Motion Magic settings
        var motionMagicConfigs = motorConfig.MotionMagic;
        // final double velo = 0.3;
        // motionMagicConfigs.MotionMagicCruiseVelocity = velo; // rps
        // motionMagicConfigs.MotionMagicAcceleration = velo * 2; // rps/s
        // motionMagicConfigs.MotionMagicJerk = (velo * 2) * 10; // rps/s/s

        motionMagicConfigs.MotionMagicCruiseVelocity = 0.3;
        motionMagicConfigs.MotionMagicAcceleration = 0.3;
        motionMagicConfigs.MotionMagicJerk = 0.3;

        OurUtils.tryApplyConfig(m_leaderMotor, motorConfig);
        OurUtils.tryApplyConfig(m_followerMotor, motorConfig);

        m_followerMotor.setControl(new Follower(m_leaderMotor.getDeviceID(), false));

        m_leaderMotor.setPosition(0);
        m_followerMotor.setPosition(0);
    }

    // TODO: this should be private and only be called on sensor trip
    public void resetMotorPositions() {
        m_leaderMotor.setPosition(0);
        m_followerMotor.setPosition(0);
    }

    private Command makeManualCommand(ElevatorManualDirection desiredDirection) {
        return startEnd(() -> {
            setDesiredControlType(DESIRED_CONTROL_TYPE.MANUAL);
            setManualDirection(desiredDirection);
        }, () -> {
            setAutomaticState(ElevatorState.IDLE);
            setManualDirection(ElevatorManualDirection.NONE);
        });
    }
    public final Command m_manualUpCommand = makeManualCommand(ElevatorManualDirection.UP);
    public final Command m_manualDownCommand = makeManualCommand(ElevatorManualDirection.DOWN);

    {
        SmartDashboard.putNumber("ELEVATOR_KG", m_kg);
        SmartDashboard.putNumber("ELEVATOR_KV", m_kv);
        SmartDashboard.putNumber("ELEVATOR_KA", m_ka);
    }

    @Override
    public void periodic() {
        {
            var kG = SmartDashboard.getNumber("ELEVATOR_KG", m_kg);
            var kV = SmartDashboard.getNumber("ELEVATOR_KV", m_kv);
            var kA = SmartDashboard.getNumber("ELEVATOR_KA", m_ka);

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

            if (apply) {
                var config = new TalonFXConfiguration();
                var leader_configurator = m_leaderMotor.getConfigurator();
                var follower_configurator = m_followerMotor.getConfigurator();

                leader_configurator.refresh(config);
                config.Slot0.kG = kG;
                config.Slot0.kV = kV;
                config.Slot0.kA = kA;

                leader_configurator.apply(config);
                follower_configurator.apply(config);
            }
        }

        if (RobotState.ENABLE_AUTOMATIC_ELEVATOR_CONTROL && m_desiredControlType == DESIRED_CONTROL_TYPE.AUTOMATIC)
            handleAutomatic();
        else
            handleManual();
            // DriverStation.reportWarning("ELEVATOR MANUAL MODE NOT ACTIVATING SINCE IT DOES NOT WORK; TUNE THE PID!!!", false); // this is if we use velocity control :skull:

        m_manualPositionPublisher.set(m_manualPosition);

        m_leaderMotorPosition.refresh();
        m_followerMotorPosition.refresh();

        m_followerMotorPositionPublisher.set(m_followerMotorPosition.getValueAsDouble());
        m_leaderMotorPositionPublisher.set(m_leaderMotorPosition.getValueAsDouble());

        m_desiredStatePublisher.set(m_state.toString());
        m_statePublisher.set(m_state.toString());
        m_positionPublisher.set(m_position.toString());
        m_manualDirectionPublisher.set(m_manualDirection.toString());
    }

    private void handleAutomatic() {
        // TODO: verify this should be IDLE
        m_state = RobotState.getCanMoveScoringMechanisms() ? m_desiredState : ElevatorState.IDLE;

        switch (m_state) {
            case HOME:
                setAutomaticPosition(ElevatorPosition.HOME);
                break;
            case IDLE:
                setAutomaticPosition(ElevatorPosition.IDLE);
                break;
            case SCORE:
                switch (RobotState.getTargetScorePosition()) {
                    case NONE:
                        setAutomaticPosition(ElevatorPosition.IDLE);
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
            case CORAL_STATION:
                setAutomaticPosition(ElevatorPosition.CORAL_STATION);
                break;
        }

        ControlRequest desiredControl = m_brake;

        switch (m_position) {
            case HOME:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.HOME_POSITION);
                break;
            case IDLE:
                desiredControl = m_brake; // redundant?
                break;
            case CORAL_STATION:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.CORAL_STATION_POSITION);
                break;

            case L1:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L1_POSITION);
                break;
            case L2:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L2_POSITION);
                break;
            case L3:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L3_POSITION);
                break;
            case L4:
                desiredControl = m_positionControl.withPosition(ElevatorConstants.L4_POSITION);
                break;
        }

        if (desiredControl instanceof MotionMagicVoltage)
            m_automaticPositionRotationsPublisher.set(((MotionMagicVoltage) desiredControl).Position);

        m_leaderMotor.setControl(desiredControl);
    }
    private void handleManual() {
        // final double increment = 0.1;

        // switch (m_manualDirection) {
        //     case NONE:
        //         setManualPosition(m_manualPosition); // we ensure this method is called for wrist clearance
        //         break;
        //     case UP:
        //         incrementManualPosition(increment);
        //         break;
        //     case DOWN:
        //         incrementManualPosition(-increment);
        //         break;
        // }

        // m_leaderMotor.setControl(m_positionControl.withPosition(m_manualPosition));
        m_leaderMotor.setControl(m_brake);
    }
}
