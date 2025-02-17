// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.Constants;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;
import frc.robot.RobotState.RelativeScorePosition;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GantrySubsystem;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface.ElevatorState;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface.GantryState;

public class AutomaticCommands {
    private static class AutomaticGoToPositionCommand extends Command {
        private final ElevatorSubsystemInterface m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        private final GantrySubsystemInterface m_gantrySubsystem = GantrySubsystem.getSingleton();

        private final RelativeScorePosition m_position;
        private final ElevatorState m_elevatorState;
        private final GantryState m_gantryState;

        public AutomaticGoToPositionCommand(RelativeScorePosition position, ElevatorState elevatorState, GantryState gantryState) {
            m_position = position;
            m_elevatorState = elevatorState;
            m_gantryState = gantryState;

            m_elevatorSubsystem.addToCommandRequirements(this);
            m_gantrySubsystem.addToCommandRequirements(this);
        }

        @Override
        public void initialize() {
            RobotState.setTargetScorePosition(m_position);

            m_elevatorSubsystem.setDesiredControlType(DESIRED_CONTROL_TYPE.AUTOMATIC);
            m_gantrySubsystem.setDesiredControlType(DESIRED_CONTROL_TYPE.AUTOMATIC);

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

    private static Command createAutomaticGoToPositionCommand(RelativeScorePosition position, ElevatorState elevatorState, GantryState gantryState) {
        return new AutomaticGoToPositionCommand(position, elevatorState, gantryState)
            .withTimeout(Constants.AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS);
    }

    public static final Command positionNONE = createAutomaticGoToPositionCommand(
        RelativeScorePosition.NONE,
        ElevatorState.IDLE,
        GantryState.IDLE
    );
    public static final Command positionCoralStation = createAutomaticGoToPositionCommand(
        RelativeScorePosition.NONE,
        ElevatorState.CORAL_STATION,
        GantryState.CORAL_STATION
    );

    private static Command createScoreCommand(RelativeScorePosition position) {
        return createAutomaticGoToPositionCommand(position, ElevatorState.SCORE, GantryState.SCORE);
    }

    public static final Command position_L1 = createScoreCommand(RelativeScorePosition.L1);
    public static final Command position_L2_L = createScoreCommand(RelativeScorePosition.L2_L);
    public static final Command position_L2_R = createScoreCommand(RelativeScorePosition.L2_R);
    public static final Command position_L3_L = createScoreCommand(RelativeScorePosition.L3_L);
    public static final Command position_L3_R = createScoreCommand(RelativeScorePosition.L3_R);
    public static final Command position_L4_L = createScoreCommand(RelativeScorePosition.L4_L);
    public static final Command position_L4_R = createScoreCommand(RelativeScorePosition.L4_R);
}