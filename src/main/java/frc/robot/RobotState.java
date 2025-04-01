// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.subsystems.CameraSubsystem;

public final class RobotState {
    // private static RobotState singleton = null;

    // public static RobotState getSingleton() {
    //     if (singleton == null)
    //         singleton = new RobotState();
    //     return singleton;
    // }


    public static Alliance ALLIANCE = null;

    public static final boolean ENABLE_AUTOMATIC_ELEVATOR_CONTROL = true;
    public static final boolean ENABLE_AUTOMATIC_GANTRY_CONTROL = true;

    public static final NetworkTableInstance NETWORK_TABLE_INSTANCE = NetworkTableInstance.getDefault();
    public static final NetworkTable robotStateTable = NETWORK_TABLE_INSTANCE.getTable("RobotState");

    private static HashMap<Long, ArrayList<Runnable>> telemetryRunnableListMap = new HashMap<>();

    public synchronized static ArrayList<Runnable> getTelemetryRunnableList(long rate) {
        var list = telemetryRunnableListMap.getOrDefault(rate, null);
        if (list == null) {
            list = new ArrayList<>();
            telemetryRunnableListMap.put(rate, list);

            new Thread(() -> {
                while (true) {
                    var lista = telemetryRunnableListMap.get(rate);
                    for (int i = 0; i < lista.size(); i++)
                        lista.get(i).run();
                    try {
                        Thread.sleep(rate);
                    } catch (InterruptedException e) { }
                }
            }, "RobotStateTelemetry" + rate + "ms").start();
        }
        return list;
    }

    public synchronized static void addTelemetry(Runnable runnable) {
        getTelemetryRunnableList(60).add(runnable);
    }
    public synchronized static void addTelemetry(Runnable runnable, long rate) {
        getTelemetryRunnableList(rate).add(runnable);
    }

    public static enum TargetScorePosition {
        NONE,
        CORAL_STATION,
        L1,

        L2_L,
        L2_R,

        L3_L,
        L3_R,

        L4_L,
        L4_R;

        public boolean getIsOnReef() {
            return !(this == NONE || this == CORAL_STATION);
        }
        public boolean getIsOnReefBranch() {
            return !(this == NONE || this == CORAL_STATION || this == L1);
        }

        public TargetScorePosition toStage1() {
            switch (this) {
                case L2_L:
                case L3_L:
                case L4_L:
                    return L2_L;
                case L2_R:
                case L3_R:
                case L4_R:
                    return L2_R;
                default:
                    return this;
            }
        }

        public TargetScorePosition toStage2() {
            switch (this) {
                case L3_L:
                case L4_L:
                    return L3_L;
                case L3_R:
                case L4_R:
                    return L3_R;
                default:
                    return this;
            }
        }
    }

    public static enum GamePieceType {
        CORAL,
        ALGAE
    }
    private static GamePieceType targetGamePieceType = GamePieceType.CORAL;
    private static final StringPublisher targetGamePieceTypePublisher = robotStateTable.getStringTopic("TargetGamePieceType").publish();

    public static synchronized GamePieceType getTargetGamePieceType() {
        return targetGamePieceType;
    }
    public static synchronized void setTargetGamePieceType(GamePieceType targetGamePieceTypeIn) {
        if (targetGamePieceType != targetGamePieceTypeIn)
            targetGamePieceTypePublisher.set(targetGamePieceTypeIn.toString());
        targetGamePieceType = targetGamePieceTypeIn;
    }
    // update nt
    static { setTargetGamePieceType(targetGamePieceType); }

    private static TargetScorePosition targetScorePosition = TargetScorePosition.NONE;
    private static final StringPublisher targetScorePositionPublisher = robotStateTable.getStringTopic("TargetScorePosition").publish();

    public static synchronized TargetScorePosition getTargetScorePosition() {
        return targetScorePosition;
    }
    public static synchronized boolean setTargetScorePosition(TargetScorePosition desiredPosition) {
        if (targetScorePosition != desiredPosition)
            targetScorePositionPublisher.set(desiredPosition.toString());
        // FIXME: ask subsystems if we can switch to this position; if we can't, don't set target position (or set it to NONE) and return false
        // ^ e.g: we are trying to score a piece, we are intaking a piece
        targetScorePosition = desiredPosition;
        return true;
    }
    // update nt
    static { setTargetScorePosition(targetScorePosition); }

    public static enum DesiredControlType {
        AUTOMATIC,
        MANUAL
    }

    public static enum IntakeState {
        IDLE,
        OUT,
        IN
    }

    private static IntakeState intakeState = IntakeState.IDLE;
    private static final StringPublisher intakeStatePublisher = robotStateTable.getStringTopic("IntakeState").publish();

    public static synchronized IntakeState getIntakeState() {
        return intakeState;
    }

    private static synchronized void setIntakeState(IntakeState desiredState) {
        if (intakeState != desiredState)
            intakeStatePublisher.set(desiredState.toString());
        intakeState = desiredState;
    }
    public static synchronized void startIntake(IntakeState intakeState) {
        setIntakeState(intakeState);
    }
    public static synchronized void stopIntake() {
        setIntakeState(IntakeState.IDLE);
    }
    // update nt
    static { setIntakeState(intakeState); }

    public static enum ClimbActuatorState {
        MANUAL,
        HOME,
        CLIMB
    }

    private static ClimbActuatorState climbActuatorState = ClimbActuatorState.HOME;
    private static final StringPublisher climbActuatorStatePublisher = robotStateTable.getStringTopic("ClimbActuatorState").publish();

    public static synchronized ClimbActuatorState getClimbActuatorState() {
        return climbActuatorState;
    }
    public static synchronized void setClimbActuatorState(ClimbActuatorState desiredState) {
        if (climbActuatorState != desiredState)
            climbActuatorStatePublisher.set(desiredState.toString());
        climbActuatorState = desiredState;
    }
    // update nt
    { setClimbActuatorState(climbActuatorState); }

    private static boolean elevatorHasClearance = true;
    private static final BooleanPublisher elevatorHasClearancePublisher = robotStateTable.getBooleanTopic("ElevatorHasClearance").publish();

    public static synchronized boolean getElevatorHasClearance() {
        return elevatorHasClearance;
    }
    public static synchronized void setElevatorHasClearance(boolean elevatorHasClearanceIn) {
        if (elevatorHasClearance != elevatorHasClearanceIn)
            elevatorHasClearancePublisher.set(elevatorHasClearance);
        elevatorHasClearance = elevatorHasClearanceIn;
    }
    // update nt
    static { setElevatorHasClearance(elevatorHasClearance); }

    private static boolean coralIsGood = false;
    private static BooleanPublisher coralIsGoodPublisher = robotStateTable.getBooleanTopic("CoralIsGood").publish();
    public static synchronized boolean getCoralIsGood() {
        return coralIsGood;
    }
    public static synchronized void setCoralIsGood(boolean coralIsGoodIn) {
        if (coralIsGood != coralIsGoodIn)
            coralIsGoodPublisher.set(coralIsGoodIn);
        coralIsGood = coralIsGoodIn;
    }
    // update nt
    static { setCoralIsGood(coralIsGood); }

    private static boolean weHaveCoral = false;
    private static BooleanPublisher weHaveCoralPublisher = robotStateTable.getBooleanTopic("WeHaveCoral").publish();
    public static synchronized boolean getWeHaveCoral() {
        return weHaveCoral;
    }
    public static synchronized void setWeHaveCoral(boolean weHaveCoralIn) {
        if (weHaveCoral != weHaveCoralIn) {
            weHaveCoralPublisher.set(weHaveCoralIn);
            if (weHaveCoral) // if we no longer have coral (weHaveCoral is old)
                setTargetScorePosition(TargetScorePosition.CORAL_STATION);
            else
                setTargetScorePosition(TargetScorePosition.L2_L);
        }
        weHaveCoral = weHaveCoralIn;
    }
    // update nt
    static { setWeHaveCoral(weHaveCoral); }

    private static boolean wantsToScoreCoral = false;

    public static synchronized boolean getWantsToScoreCoral() {
        return wantsToScoreCoral;
    }
    public static synchronized void setWantsToScoreCoral(boolean wantsToScoreCoralIn) {
        wantsToScoreCoral = wantsToScoreCoralIn;
    }

    private static boolean wantsToScoreAlgae = false;

    public static synchronized boolean getWantsToScoreAlgae() {
        return wantsToScoreAlgae;
    }
    public static synchronized void setWantsToScoreAlgae(boolean wantsToScoreAlgaeIn) {
        wantsToScoreAlgae = wantsToScoreAlgaeIn;
    }

    private static Optional<Double> reefTargetHorizontalDistance = Optional.empty();
    public static synchronized Optional<Double> getReefTargetHorizontalDistance() {
        return reefTargetHorizontalDistance;
    }
    public static synchronized void setReefTargetHorizontalDistance(double distance) {
        reefTargetHorizontalDistance = Optional.of(distance);
    }
    public static synchronized void clearReefTargetHorizontalDistance() {
        reefTargetHorizontalDistance = Optional.empty();
    }

    private static Optional<Double> reefTargetForwardDistance = Optional.empty();
    public static synchronized Optional<Double> getReefTargetForwardDistance() {
        return reefTargetForwardDistance;
    }
    public static synchronized void setReefTargetForwardDistance(double distance) {
        reefTargetForwardDistance = Optional.of(distance);
    }
    public static synchronized void clearReefTargetForwardDistance() {
        reefTargetForwardDistance = Optional.empty();
    }

    private static boolean visionPoseSuccess = false;

    public static synchronized void setVisionPoseSuccess(boolean newValue) {
        visionPoseSuccess = newValue;
    }
    public static synchronized boolean getVisionPoseSuccess() {
        return visionPoseSuccess;
    }

    private static double driveSpeed = 0;
    private static final DoublePublisher driveSpeedPublisher = robotStateTable.getDoubleTopic("DriveSpeed").publish();
    public static synchronized void setDriveSpeed(double speed) {
        if (driveSpeed != speed)
            driveSpeedPublisher.set(speed);
        driveSpeed = speed;
    }
    // update NT
    static { setDriveSpeed(driveSpeed); }

    public static Rotation2d initialSwerveRotation = null;

    public static synchronized void updateState(RobotContainer robotContainer) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() != ALLIANCE) {
            ALLIANCE = alliance.get();
            AprilTagConstants.update(ALLIANCE);

            // make sure forward faces red alliance wall
            if (ALLIANCE == Alliance.Red)
                initialSwerveRotation = Rotation2d.kZero;
            else
                initialSwerveRotation = Rotation2d.k180deg;

            CameraSubsystem.update();
            for (var value : CameraSubsystem.RelativeReefLocation.values())
                value.update();
            for (var value : CameraSubsystem.CoralStationID.values())
                value.update();
            if (robotContainer != null) {
                robotContainer.update(initialSwerveRotation);
            }
        }
    }
}
