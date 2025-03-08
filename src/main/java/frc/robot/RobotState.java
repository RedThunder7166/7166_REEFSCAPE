// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    public static enum TargetScorePosition {
        NONE,
        CORAL_STATION,
        L1,

        L2_L,
        L2_R,

        L3_L,
        L3_R,

        L4_L,
        L4_R
    }

    private static TargetScorePosition targetScorePosition = TargetScorePosition.NONE;
    private static final StringPublisher targetScorePositionPublisher = robotStateTable.getStringTopic("TargetScorePosition").publish();

    public static synchronized TargetScorePosition getTargetScorePosition() {
        return targetScorePosition;
    }
    public static synchronized boolean setTargetScorePosition(TargetScorePosition desiredPosition) {
        // FIXME: ask subsystems if we can switch to this position; if we can't, don't set target position (or set it to NONE) and return false
        // ^ e.g: we are trying to score a piece, we are intaking a piece
        targetScorePosition = desiredPosition;
        targetScorePositionPublisher.set(targetScorePosition.toString());
        return true;
    }
    // update nt
    { setTargetScorePosition(targetScorePosition); }

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

    private static synchronized void setIntakeState(IntakeState intakeStateIn) {
        intakeState = intakeStateIn;
        intakeStatePublisher.set(intakeState.toString());
    }
    public static synchronized void startIntake(IntakeState intakeState) {
        setIntakeState(intakeState);
    }
    public static synchronized void stopIntake() {
        setIntakeState(IntakeState.IDLE);
    }
    // update nt
    { setIntakeState(intakeState); }

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
    public static synchronized void setClimbActuatorState(ClimbActuatorState climbStateIn) {
        climbActuatorState = climbStateIn;
        climbActuatorStatePublisher.set(climbActuatorState.toString());
    }
    // update nt
    { setClimbActuatorState(climbActuatorState); }

    private static boolean elevatorHasClearance = true;
    private static final BooleanPublisher elevatorHasClearancePublisher = robotStateTable.getBooleanTopic("ElevatorHasClearance").publish();

    public static synchronized boolean getElevatorHasClearance() {
        return elevatorHasClearance;
    }
    public static synchronized void setElevatorHasClearance(boolean elevatorHasClearanceIn) {
        elevatorHasClearance = elevatorHasClearanceIn;
        elevatorHasClearancePublisher.set(elevatorHasClearance);
    }
    // update nt
    { setElevatorHasClearance(elevatorHasClearance); }

    private static boolean coralIsGood = false;
    public static synchronized boolean getCoralIsGood() {
        return coralIsGood;
    }
    public static synchronized void setCoralIsGood(boolean coralIsGoodIn) {
        coralIsGood = coralIsGoodIn;
    }

    private static boolean wantsToScore = false;

    public static synchronized boolean getWantsToScore() {
        return wantsToScore;
    }
    public static synchronized void setWantsToScore(boolean wantsToScoreIn) {
        wantsToScore = wantsToScoreIn;
    }

    public static final double reefTargetHorizontalDistanceOffset = Units.inchesToMeters(2) - 0.011; // 0.056

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
