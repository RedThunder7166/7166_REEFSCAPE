// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GantryConstants;
import frc.robot.Constants.IntakeOuttakeConstants;
import frc.robot.OurUtils;
import frc.robot.RobotState;

public class IntakeOuttakeSubsystem extends SubsystemBase {
    private static IntakeOuttakeSubsystem singleton = null;

    public static IntakeOuttakeSubsystem getSingleton() {
        if (singleton == null)
            singleton = new IntakeOuttakeSubsystem();
        return singleton;
    }

    private final RobotState m_robotState = RobotState.getSingleton();

    private final TalonFX m_motor = new TalonFX(IntakeOuttakeConstants.MOTOR_ID);
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();

    public IntakeOuttakeSubsystem() {
        // FIXME: tune intake outtake PID
        TalonFXConfiguration configs = new TalonFXConfiguration();

        /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        configs.Slot0.kI = 0; // No output for integrated error
        configs.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        configs.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        OurUtils.tryApplyConfig(m_motor, configs);
    }

    @Override
    public void periodic() {
        ControlRequest targetRequest = m_brake;
        switch (m_robotState.getIntakeState()) {
            case IDLE:
                targetRequest = m_brake; // redundant?
                break;
            case FORWARD:
                targetRequest = m_velocityVoltage.withVelocity(IntakeOuttakeConstants.FORWARD_VELOCITY);
                break;
            case BACKWARD:
                targetRequest = m_velocityVoltage.withVelocity(IntakeOuttakeConstants.BACKWARD_VELOCITY);
                break;
        }

        m_motor.setControl(targetRequest);
    }

    private Command makeCommand(boolean isForward) {
        return this.startEnd(() -> {
            m_robotState.startIntake(isForward);
        }, () -> {
            m_robotState.stopIntake();
        });
    }
    public final Command m_forwardCommand = makeCommand(true);
    public final Command m_backwardCommand = makeCommand(false);
}
