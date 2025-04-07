// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.random.RandomGenerator;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.GantryConstants;
import frc.robot.subsystems.CameraSubsystem.RelativeReefLocation;

public final class OurUtils {
    // Phoenix 6 methods:

    public static void tryApplyConfig(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = motor.getConfigurator().apply(config, 100);
            if (status.isOK()) break;
        }
        if (!status.isOK())
            DriverStation.reportWarning("Could not apply configs to device " + motor.getDeviceID() + "; error code: " + status.toString(), false);
    }
    public static void tryApplyConfig(CANcoder encoder, CANcoderConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = encoder.getConfigurator().apply(config, 100);
            if (status.isOK()) break;
        }
        if (!status.isOK())
            DriverStation.reportWarning("Could not apply configs to device " + encoder.getDeviceID() + "; error code: " + status.toString(), false);
    }
    public static void tryApplyConfig(CANrange sensor, CANrangeConfiguration config) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = sensor.getConfigurator().apply(config, 100);
            if (status.isOK()) break;
        }
        if (!status.isOK())
            DriverStation.reportWarning("Could not apply configs to device " + sensor.getDeviceID() + "; error code: " + status.toString(), false);
    }

    // Phoenix 5 methods:

    public static void tryApplyConfig(CANdle candle, int deviceID, CANdleConfiguration config) {
        ErrorCode status = ErrorCode.GENERAL_ERROR;
        for (int i = 0; i < 5; ++i) {
            status = candle.configAllSettings(config, 100);
            if (status == ErrorCode.OK) break;
        }
        if (status != ErrorCode.OK)
            DriverStation.reportWarning("Could not apply configs to device " + deviceID + "; error code: " + status.toString(), false);
    }

    public static String formatReefLocation(RelativeReefLocation location) {
        if (location == null)
            return "NULL";

        return location.toString() + '_' + location.getTagID();
    }

    public static interface DIOInterface {
        public boolean get();
    }
    public static final class FakeDIO implements DIOInterface {
        private final int m_channel;
        private boolean m_value = true;

        public FakeDIO(int channel) {
            m_channel = channel;
        }
        public void set(boolean value) {
            m_value = value;
        }
        public boolean get() {
            switch (m_channel) {
                case GantryConstants.ELEVATOR_CLEARANCE_SENSOR_ID:
                    return false;
                case GantryConstants.SCORE_EXIT_SENSOR_ID:
                    return RandomGenerator.getDefault().nextBoolean();
                default:
                    return m_value;
            }
        }
    }

    public static final class WrappedDIO implements DIOInterface {
        private final DigitalInput m_digitalInput;
        public WrappedDIO(DigitalInput digitalInput) {
            m_digitalInput = digitalInput;
        }

        public boolean get() {
            return m_digitalInput.get();
        }
    }

    public static DIOInterface getDIO(int channel) {
        if (Robot.isSimulation())
            return new FakeDIO(channel);
        return new WrappedDIO(new DigitalInput(channel));
    }

    // from arduino
    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}