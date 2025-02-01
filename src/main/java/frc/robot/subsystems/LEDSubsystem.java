// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OurUtils;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(LEDConstants.CANDLE_ID);
    public LEDSubsystem() {
        // FIXME: CANdle configs
        final CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1;

        OurUtils.tryApplyConfig(m_candle, LEDConstants.CANDLE_ID, config);
    }

    @Override
    public void periodic() {
        
    }
}
