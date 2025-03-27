// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.Constants;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.RobotState.TargetScorePosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GantrySubsystem;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface.ElevatorState;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface.GantryState;

public class AutomaticCommands {
    private static TargetScorePosition targetScorePosition = RobotState.getTargetScorePosition();

    public static TargetScorePosition getTargetScorePosition() {
        return targetScorePosition;
    }

    private static final boolean ADD_COMMANDS_TO_REQUIREMENTS = false;

    private static class InstantAutomaticGoToPositionCommand extends Command {
        private TargetScorePosition m_targetScorePosition;

        private final ElevatorSubsystemInterface m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        private final GantrySubsystemInterface m_gantrySubsystem = GantrySubsystem.getSingleton();

        private final ElevatorState m_elevatorState;
        private final GantryState m_gantryState;

        public InstantAutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
            m_elevatorState = elevatorState;
            m_gantryState = gantryState;

            if (ADD_COMMANDS_TO_REQUIREMENTS) {
                m_elevatorSubsystem.addToCommandRequirements(this);
                m_gantrySubsystem.addToCommandRequirements(this);
            }
        }

        @Override
        public void initialize() {
            m_targetScorePosition = targetScorePosition;
            RobotState.setTargetScorePosition(m_targetScorePosition);
            RobotState.stopIntake();

            m_elevatorSubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);
            m_gantrySubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);

            m_elevatorSubsystem.setAutomaticState(m_elevatorState);
            m_gantrySubsystem.setAutomaticState(m_gantryState);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    private static class AutomaticGoToPositionCommand extends Command {
        private TargetScorePosition m_targetScorePosition;

        private final ElevatorSubsystemInterface m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        private final GantrySubsystemInterface m_gantrySubsystem = GantrySubsystem.getSingleton();

        private final ElevatorState m_elevatorState;
        private final GantryState m_gantryState;

        private boolean m_isTeleop = false;
        private boolean m_hasDoneAction = false;

        private boolean m_coralHasBeenGood = false;
        private boolean m_elevatorHasBeenAligned = false;
        private boolean m_alreadyTargeting = false;

        private Timer m_timer = new Timer();

        public AutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
            m_elevatorState = elevatorState;
            m_gantryState = gantryState;

            if (ADD_COMMANDS_TO_REQUIREMENTS) {
                m_elevatorSubsystem.addToCommandRequirements(this);
                m_gantrySubsystem.addToCommandRequirements(this);
            }
        }

        @Override
        public void initialize() {
            m_hasDoneAction = false;
            m_targetScorePosition = targetScorePosition;
            m_alreadyTargeting = m_elevatorSubsystem.getIsTargetingAScoreLocation() && RobotState.getTargetScorePosition().getIsOnReef();
            m_isTeleop = DriverStation.isTeleop();
            m_coralHasBeenGood = m_isTeleop;
            m_elevatorHasBeenAligned = m_isTeleop;
            m_timer.restart();
        }

        @Override
        public void execute() {
            if (!m_elevatorHasBeenAligned)
                m_elevatorHasBeenAligned = m_elevatorSubsystem.getIsAtPosition(m_targetScorePosition);

            if (!m_hasDoneAction && m_coralHasBeenGood) {
                m_hasDoneAction = true;
                RobotState.setTargetScorePosition(m_targetScorePosition);
                RobotState.stopIntake();

                m_elevatorSubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);
                m_gantrySubsystem.setDesiredControlType(DesiredControlType.AUTOMATIC);

                m_elevatorSubsystem.setAutomaticState(m_elevatorState);
                m_gantrySubsystem.setAutomaticState(m_gantryState);
            } else if (RobotState.getCoralIsGood())
                m_coralHasBeenGood = true;
        }

        @Override
        public boolean isFinished() {
            if (m_hasDoneAction && m_isTeleop)
                return true;

            if (!m_coralHasBeenGood)
                return false;
            if (m_alreadyTargeting) {
                if (!m_hasDoneAction)
                    return false;
            } else if (!m_timer.hasElapsed(0.75))
                return false;

            return m_elevatorHasBeenAligned && m_gantrySubsystem.getIsAtTargetPosition();
        }
    }


    private static Command createAutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
        return new AutomaticGoToPositionCommand(elevatorState, gantryState);
            // .withTimeout(Constants.AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS);
    }
    private static Command createInstantAutomaticGoToPositionCommand(ElevatorState elevatorState, GantryState gantryState) {
        return new InstantAutomaticGoToPositionCommand(elevatorState, gantryState);
    }

    public static Command createSetTargetScorePositionCommand(TargetScorePosition targetScorePositionIn) {
        return new InstantCommand(() -> targetScorePosition = targetScorePositionIn);
    }

    public static Command createGoToPositionCommand() {
        return createAutomaticGoToPositionCommand(ElevatorState.TARGET, GantryState.TARGET);
    }
    public static Command createInstantGoToPositionCommand() {
        return createInstantAutomaticGoToPositionCommand(ElevatorState.TARGET, GantryState.TARGET);
    }

    public static Command createGoToPositionCommand(TargetScorePosition position) {
        return createSetTargetScorePositionCommand(position).andThen(createGoToPositionCommand());
    }
    public static Command createInstantGoToPositionCommand(TargetScorePosition position) {
        return createSetTargetScorePositionCommand(position).andThen(createInstantGoToPositionCommand());
    }

    public static final Command positionNONE = createInstantAutomaticGoToPositionCommand(
        ElevatorState.IDLE,
        GantryState.IDLE
    );
}