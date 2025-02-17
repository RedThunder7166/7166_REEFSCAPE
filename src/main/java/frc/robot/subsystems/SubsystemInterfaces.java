// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;

public class SubsystemInterfaces {
    private static interface GenericInterface {
        public Command addToCommandRequirements(Command command);
    }

    public static interface ElevatorSubsystemInterface extends GenericInterface {
        public static enum ElevatorState {
            HOME,
            IDLE,
            SCORE,
            CORAL_STATION
        }

        public void setAutomaticState(ElevatorState desiredState);

        public void setIdle();

        public static enum ElevatorManualDirection {
            NONE,
            UP,
            DOWN
        }
        
        public void setManualDirection(ElevatorManualDirection desiredManualDirection);

        public void incrementManualPosition(double value);
        public void resetManualPosition();

        public void setDesiredControlType(DESIRED_CONTROL_TYPE desiredControlType);

        public boolean getIsAtTargetPosition();

        public Command getManualUpCommand();
        public Command getManualDownCommand();

        public Command getTempGoUntilTargetIncreaseCommand(double targetIncreaseInPosition);
        public Command getTempHoldPositionCommand();
        public Command getTimeTravelCommand(double targetPosition);
    }

    public static interface GantrySubsystemInterface extends GenericInterface {
        public static enum GantryState {
            IDLE,
            SCORE,
            CORAL_STATION
        }
        public void setAutomaticState(GantryState desiredState);

        public void setIdle();

        public static enum GantryManualDirection {
            NONE,
            LEFT,
            RIGHT
        }
        public void setManualDirection(GantryManualDirection desiredManualDirection);

        public void incrementManualPosition(double value);
        public void resetManualPosition();

        public void setDesiredControlType(DESIRED_CONTROL_TYPE desiredControlType);

        public void resetMotorPosition();

        public boolean getIsAtTargetPosition();

        public Command getManualLeftCommand();
        public Command getManualRightCommand();
    }

    public static interface IntakeOuttakeSubsystemInterface extends GenericInterface {
        public Command getOutCommand();
        public Command getInCommand();
    }
}