// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.StackWalker.Option;
import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
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

    public static TargetScorePosition getTargetScorePosition() {
        return targetScorePosition;
    }
    public static boolean setTargetScorePosition(TargetScorePosition desiredPosition) {
        // FIXME: ask subsystems if we can switch to this position; if we can't, don't set target position (or set it to NONE) and return false
        // ^ e.g: we are trying to score a piece, we are intaking a piece
        targetScorePosition = desiredPosition;
        targetScorePositionPublisher.set(targetScorePosition.toString());
        return true;
    }

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

    public static IntakeState getIntakeState() {
        return intakeState;
    }

    private static void setIntakeState(IntakeState intakeStateIn) {
        intakeState = intakeStateIn;
        intakeStatePublisher.set(intakeState.toString());
    }
    public static void startIntake(boolean isForward) {
        setIntakeState(isForward ? IntakeState.OUT : IntakeState.IN);
    }
    public static void stopIntake() {
        setIntakeState(IntakeState.IDLE);
    }

    private static boolean elevatorHasClearance = true;
    private static final BooleanPublisher elevatorHasClearancePublisher = robotStateTable.getBooleanTopic("ElevatorHasClearance").publish();

    public static boolean getElevatorHasClearance() {
        return elevatorHasClearance;
    }
    public static void setElevatorHasClearance(boolean elevatorHasClearanceIn) {
        elevatorHasClearance = elevatorHasClearanceIn;
        elevatorHasClearancePublisher.set(elevatorHasClearance);
    }

    private static boolean coralIsGood = false;
    public static boolean getCoralIsGood() {
        return coralIsGood;
    }
    public static void setCoralIsGood(boolean coralIsGoodIn) {
        coralIsGood = coralIsGoodIn;
    }

    private static boolean wantsToScore = false;

    public static boolean getWantsToScore() {
        return wantsToScore;
    }
    public static void setWantsToScore(boolean wantsToScoreIn) {
        wantsToScore = wantsToScoreIn;
    }

    public static final double reefTargetHorizontalDistanceOffset = 0; // 0.056

    private static Optional<Double> reefTargetHorizontalDistance = Optional.empty();
    public static Optional<Double> getReefTargetHorizontalDistance() {
        return reefTargetHorizontalDistance;
    }
    public static void setReefTargetHorizontalDistance(double distance) {
        reefTargetHorizontalDistance = Optional.of(distance);
    }
    public static void clearReefTargetHorizontalDistance() {
        reefTargetHorizontalDistance = Optional.empty();
    }

    public static Rotation2d initialSwerveRotation = null;

    public static void updateState(RobotContainer robotContainer) {
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
            if (robotContainer != null) {
                robotContainer.update(initialSwerveRotation);
            }
        }
    }
}
