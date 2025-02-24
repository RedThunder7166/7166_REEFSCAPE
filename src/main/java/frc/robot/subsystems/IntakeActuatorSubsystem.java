// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeActuatorConstants;
import frc.robot.OurUtils;
import frc.robot.subsystems.SubsystemInterfaces.IntakeActuatorSubsystemInterface;

public class IntakeActuatorSubsystem extends SubsystemBase implements IntakeActuatorSubsystemInterface {
    private static final class FakeIntakeActuatorSubsystem implements IntakeActuatorSubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }
    }

    private static IntakeActuatorSubsystemInterface singleton = null;

    public static IntakeActuatorSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = IntakeActuatorConstants.REAL ? new IntakeActuatorSubsystem() : new FakeIntakeActuatorSubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private final TalonFX m_motor = new TalonFX(IntakeActuatorConstants.MOTOR_ID);
    // private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    public IntakeActuatorSubsystem() {
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        configs.Feedback.SensorToMechanismRatio = (54d / 12d) * (54 / 18d) * (240d / 10d);

        configs.MotionMagic.MotionMagicAcceleration = 0.3;
        configs.MotionMagic.MotionMagicCruiseVelocity = 0.3;

        OurUtils.tryApplyConfig(m_motor, configs);

        m_motor.setPosition(0);
    }
    
    @Override
    public void periodic() {
        ControlRequest targetRequest = m_brake;

        m_motor.setControl(targetRequest);
    }
}
