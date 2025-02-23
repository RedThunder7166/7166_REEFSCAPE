// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.SubsystemInterfaces.IntakeOuttakeSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface.GantryState;

public class AutomaticCommands {
    private static class AutomaticGoToPositionCommand extends Command {
        private final ElevatorSubsystemInterface m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        private final GantrySubsystemInterface m_gantrySubsystem = GantrySubsystem.getSingleton();

        private final TargetScorePosition m_position;
        private final ElevatorState m_elevatorState;
        private final GantryState m_gantryState;

        public AutomaticGoToPositionCommand(TargetScorePosition position, ElevatorState elevatorState, GantryState gantryState) {
            m_position = position;
            m_elevatorState = elevatorState;
            m_gantryState = gantryState;

            m_elevatorSubsystem.addToCommandRequirements(this);
            m_gantrySubsystem.addToCommandRequirements(this);
        }

        @Override
        public void initialize() {
            RobotState.setTargetScorePosition(m_position);

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

    private static Command createAutomaticGoToPositionCommand(TargetScorePosition position, ElevatorState elevatorState, GantryState gantryState) {
        return new AutomaticGoToPositionCommand(position, elevatorState, gantryState)
            .withTimeout(Constants.AUTOMATIC_GO_TO_POSITION_TIMEOUT_SECONDS);
    }

    public static final Command positionNONE = createAutomaticGoToPositionCommand(
        TargetScorePosition.NONE,
        ElevatorState.IDLE,
        GantryState.IDLE
    );
    public static final Command positionCoralStation = createAutomaticGoToPositionCommand(
        TargetScorePosition.CORAL_STATION,
        ElevatorState.TARGET,
        GantryState.TARGET
    );

    public static Command createScoreCommand(TargetScorePosition position) {
        return createAutomaticGoToPositionCommand(
            position,
            ElevatorState.TARGET,
            GantryState.TARGET
        );
    }

    public static Command createLocalizeToReefCommand(RelativeReefLocation targetLocation) {
        return new CameraSubsystem.DynamicCommand(() -> {
            return CameraSubsystem.getSingleton().getPathCommandFromReefTag(targetLocation);
        });
    }

    public static class IntakeCommand extends Command {
        IntakeOuttakeSubsystemInterface m_intakeOuttakeSubsystem;
        public IntakeCommand(IntakeOuttakeSubsystemInterface intakeOuttakeSubsystem) {
            m_intakeOuttakeSubsystem = intakeOuttakeSubsystem;

            m_intakeOuttakeSubsystem.addToCommandRequirements(this);
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean isInterrupted) {
            
        }
    }
}