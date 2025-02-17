// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    private final TalonFX m_scoreMotor = new TalonFX(IntakeOuttakeConstants.SCORE_MOTOR_ID);
    private final TalonFX m_intakeMotor = new TalonFX(IntakeOuttakeConstants.INTAKE_MOTOR_ID);
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0);
    private final NeutralOut m_brake = new NeutralOut();

    public IntakeOuttakeSubsystem() {
        // FIXME: tune intake outtake PID
        TalonFXConfiguration configs = new TalonFXConfiguration();

        OurUtils.tryApplyConfig(m_scoreMotor, configs);

        // TODO: intakeMotor should be controlled entirely through sensors, so remove this
        m_intakeMotor.setControl(new Follower(m_scoreMotor.getDeviceID(), true));
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

        m_scoreMotor.setControl(targetRequest);
    }

    private Command makeCommand(boolean isForward) {
        return this.startEnd(() -> {
            RobotState.startIntake(isForward);
        }, () -> {
            RobotState.stopIntake();
        });
    }
    public final Command m_outCommand = makeCommand(true);
    public final Command m_inCommand = makeCommand(false);
}
