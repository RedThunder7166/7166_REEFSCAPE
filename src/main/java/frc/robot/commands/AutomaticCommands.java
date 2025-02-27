// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.Constants;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.RobotState.TargetScorePosition;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GantrySubsystem;
import frc.robot.subsystems.CameraSubsystem.RelativeReefLocation;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface.ElevatorState;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface.GantryState;

public class AutomaticCommands {
    private static TargetScorePosition m_targetScorePosition = RobotState.getTargetScorePosition();

    private static class AutomaticGoToPositionCommand extends Command {
        private final ElevatorSubsystemInterface m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        private final GantrySubsystemInterface m_gantrySubsystem = GantrySubsystem.getSingleton();

        private final ElevatorState m_elevatorState;
        private final GantryState m_gantryState;

        public AutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
            m_elevatorState = elevatorState;
            m_gantryState = gantryState;

            m_elevatorSubsystem.addToCommandRequirements(this);
            m_gantrySubsystem.addToCommandRequirements(this);
        }

        @Override
        public void initialize() {
            RobotState.setTargetScorePosition(m_targetScorePosition);

            m_elevatorSubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);
            m_gantrySubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);

            m_elevatorSubsystem.setAutomaticState(m_elevatorState);
            m_gantrySubsystem.setAutomaticState(m_gantryState);
        }

        @Override
        public void execute() { }

        @Override
        public boolean isFinished() {
            return m_elevatorSubsystem.getIsAtTargetPosition() && m_gantrySubsystem.getIsAtTargetPosition();
        }

        @Override
        public void end(boolean isInterrupted) { }
    }


    private static Command createAutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
        return new AutomaticGoToPositionCommand(elevatorState, gantryState)
            .withTimeout(Constants.AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS);
    }

    public static Command createSetTargetScorePositionCommand(TargetScorePosition targetScorePosition) {
        return new InstantCommand(() -> {
            m_targetScorePosition = targetScorePosition;
        });
    }
    public static Command createGoToPositionCommand() {
        return createAutomaticGoToPositionCommand(ElevatorState.TARGET, GantryState.TARGET);
    }

    public static Command createSetAndGoToTargetScorePositionCommand(TargetScorePosition position) {
        return createSetTargetScorePositionCommand(position).andThen(createGoToPositionCommand());
    }

    public static final Command positionNONE = createAutomaticGoToPositionCommand(
        ElevatorState.IDLE,
        GantryState.IDLE
    );

    public static Command createLocalizeToReefCommand(RelativeReefLocation targetLocation) {
        return new CameraSubsystem.DynamicCommand(() -> {
            return CameraSubsystem.getSingleton().getPathCommandFromReefTag(targetLocation);
        });
    }

    private static class TESTAutomaticGoToPositionCommand extends Command {
        private final ElevatorSubsystemInterface m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        private final GantrySubsystemInterface m_gantrySubsystem = GantrySubsystem.getSingleton();

        private final ElevatorState m_elevatorState;
        private final GantryState m_gantryState;

        private boolean m_started = false;

        public TESTAutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
            m_elevatorState = elevatorState;
            m_gantryState = gantryState;

            m_elevatorSubsystem.addToCommandRequirements(this);
            m_gantrySubsystem.addToCommandRequirements(this);
        }

        @Override
        public void initialize() { }

        @Override
        public void execute() {
            if (!m_started && RobotState.getCoralIsGood()) {
                m_started = true;

                RobotState.stopIntake();
                RobotState.setTargetScorePosition(m_targetScorePosition);

                m_elevatorSubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);
                m_gantrySubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);

                m_elevatorSubsystem.setAutomaticState(m_elevatorState);
                m_gantrySubsystem.setAutomaticState(m_gantryState);
            }
        }

        @Override
        public boolean isFinished() {
            return m_started && m_elevatorSubsystem.getIsAtTargetPosition() && m_gantrySubsystem.getIsAtTargetPosition();
        }

        @Override
        public void end(boolean isInterrupted) { }
    }


    private static Command TESTcreateAutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
        return new TESTAutomaticGoToPositionCommand(elevatorState, gantryState)
            .withTimeout(Constants.AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS);
    }

    public static Command TESTcreateGoToPositionCommand() {
        return TESTcreateAutomaticGoToPositionCommand(ElevatorState.TARGET, GantryState.TARGET);
    }
}