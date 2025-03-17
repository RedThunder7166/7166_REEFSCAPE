// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.DesiredControlType;

public class SubsystemInterfaces {
    private static interface GenericInterface {
        public Command addToCommandRequirements(Command command);
    }

    public static enum GenericDirection {
        NONE,
        OUT,
        IN
    }

    public static interface ElevatorSubsystemInterface extends GenericInterface {
        public static enum ElevatorState {
            HOME,
            IDLE,
            TARGET,
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

        public boolean getIsAtTargetPosition();
        public boolean getIsBelowL3();

        public Command getManualUpCommand();
        public Command getManualDownCommand();

        public Command getTempGoUntilTargetIncreaseCommand(double targetIncreaseInPosition);
        public Command getTempHoldPositionCommand();
        public Command getTimeTravelCommand(double targetPosition);
    }

    public static interface GantrySubsystemInterface extends GenericInterface {
        public static enum GantryState {
            IDLE,
            TARGET,
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

        public boolean getIsAtTargetPosition();
        public boolean getScoreEnterSensorTripped();
        public boolean getScoreExitSensorTripped();

        public Command getManualLeftCommand();
        public Command getManualRightCommand();

        public Command getGantryResetPositionCommand();
    }

    public static interface IntakeOuttakeSubsystemInterface extends GenericInterface {
    }
    public static interface ClimbSubsystemInterface extends GenericInterface {
        public void incrementManualPosition(double value);
        public void resetManualPosition();

        public Command getManualActuatorOutCommand();
        public Command getManualActuatorInCommand();

        public Command getCageOutCommand();
        public Command getCageInCommand();

        public Command getAutomaticCageHomeCommand();
        public Command getAutomaticCageOutCommand();
    }

    public static interface AlgaeHandSubsystemInterface extends GenericInterface {
        public boolean isTargetingManualOut();

        public Command getManualOutCommand();
        public Command getManualInCommand();

        public Command getMiddleCommand();
    }
}