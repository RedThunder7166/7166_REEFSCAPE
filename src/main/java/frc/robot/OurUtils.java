// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DriverStation;

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
}