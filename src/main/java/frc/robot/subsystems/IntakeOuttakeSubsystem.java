// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeOuttakeConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.subsystems.SubsystemInterfaces.IntakeOuttakeSubsystemInterface;

public class IntakeOuttakeSubsystem extends SubsystemBase implements IntakeOuttakeSubsystemInterface {
    private static final class FakeIntakeOuttakeSubsystem implements IntakeOuttakeSubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }
    }

    private static IntakeOuttakeSubsystemInterface singleton = null;

    public static IntakeOuttakeSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = IntakeOuttakeConstants.REAL ? new IntakeOuttakeSubsystem() : new FakeIntakeOuttakeSubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private final TalonFX m_motor = new TalonFX(IntakeOuttakeConstants.INTAKE_MOTOR_ID);
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    public IntakeOuttakeSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        OurUtils.tryApplyConfig(m_motor, configs);
    }
    
    @Override
    public void periodic() {
        ControlRequest targetRequest = m_brake;
        switch (RobotState.getIntakeState()) {
            case IDLE:
                targetRequest = m_brake; // redundant?
                break;
            case OUT:
                targetRequest = m_dutyCycleOut.withOutput(IntakeOuttakeConstants.BACKWARD_OUTPUT);
                break;
            case IN:
                targetRequest = m_dutyCycleOut.withOutput(IntakeOuttakeConstants.FORWARD_OUTPUT);
                break;
        }

        m_motor.setControl(targetRequest);
    }
}
