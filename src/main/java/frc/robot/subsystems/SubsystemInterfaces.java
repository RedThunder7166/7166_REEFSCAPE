// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.DesiredControlType;
import frc.robot.RobotState.TargetScorePosition;

public class SubsystemInterfaces {
    private static interface GenericSubsystemInterface {
        public Command addToCommandRequirements(Command command);
    }

    public static enum GenericDirection {
        NONE,
        OUT,
        IN
    }

    public static interface LEDSubsystemInterface extends GenericSubsystemInterface {

    }

    public static interface ElevatorSubsystemInterface extends GenericSubsystemInterface {
        public static enum ElevatorState {
            HOME,
            IDLE,
            TARGET
        }

        public void setAutomaticState(ElevatorState desiredState);

        public void setIdle();

        public static enum ElevatorManualDirection {
            NONE,
            UP,
            DOWN
        }

        public void incrementManualPosition(double value);
        public void resetManualPosition();

        public void setDesiredControlType(DesiredControlType desiredControlType);

        public boolean getIsAtPosition(TargetScorePosition position);

        public boolean getIsAtTargetPosition();
        public boolean getIsTargetingAScoreLocation();
        public boolean getIsBelowL3();

        public Command getManualUpCommand();
        public Command getManualDownCommand();

        public void manualHoldThisPosition();

        public Command getTempGoUntilTargetIncreaseCommand(double targetIncreaseInPosition);
        public Command getTempHoldPositionCommand();
        public Command getTimeTravelCommand(double targetPosition);
    }

    public static interface GantrySubsystemInterface extends GenericSubsystemInterface {
        public static enum GantryState {
            IDLE,
            TARGET
        }
        public void setAutomaticState(GantryState desiredState);

        public void setIdle();

        public static enum GantryManualDirection {
            NONE,
            LEFT,
            RIGHT
        }

        public void incrementManualPosition(double value);
        public void resetManualPosition();

        public void setDesiredControlType(DesiredControlType desiredControlType);

        public void resetMotorPosition();

        public void resetPositionStuff();

        public boolean getIsTargetingAScoreLocation();

        public boolean getIsAtPosition(TargetScorePosition position);

        public boolean getIsAtTargetPosition();
        public boolean getScoreEnterSensorTripped();
        public boolean getScoreExitSensorTripped();

        public Command getManualLeftCommand();
        public Command getManualRightCommand();

        public void manualSeedThisPosition();

        public Command getGantryResetPositionCommand();
    }

    public static interface IntakeOuttakeSubsystemInterface extends GenericSubsystemInterface {
    }
    public static interface ClimbSubsystemInterface extends GenericSubsystemInterface {
        public void incrementManualPosition(double value);
        public void resetManualPosition();

        public Command getManualActuatorOutCommand();
        public Command getManualActuatorInCommand();

        public Command getCageOutCommand();
        public Command getCageInCommand();

        public Command getAutomaticCageHomeCommand();
        public Command getAutomaticCageOutCommand();
    }

    public static interface AlgaeHandSubsystemInterface extends GenericSubsystemInterface {
        public boolean isTargetingManualOut();

        public Command getManualOutCommand();
        public Command getManualInCommand();

        public Command getRetractCommand();
        public Command getExtendedCommand();
    }

    public static interface AlgaeMouthSubsystemInterface extends GenericSubsystemInterface {
        public Command getManualArmOutCommand();
        public Command getManualArmInCommand();
    }
}