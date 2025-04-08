// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OurUtils;
import frc.robot.RobotState;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.SubsystemInterfaces.LEDSubsystemInterface;

public class LEDSubsystem extends SubsystemBase implements LEDSubsystemInterface {
    private static final class FakeLEDSubsystem implements LEDSubsystemInterface {
        @Override
        public Command addToCommandRequirements(Command command) {
            return command;
        }
    }

    private static LEDSubsystemInterface singleton = null;

    public static synchronized LEDSubsystemInterface getSingleton() {
        if (singleton == null)
            singleton = LEDConstants.REAL ? new LEDSubsystem() : new FakeLEDSubsystem();
        return singleton;
    }

    @Override
    public Command addToCommandRequirements(Command command) {
        command.addRequirements(this);
        return command;
    }

    private final CANdle m_candle = new CANdle(LEDConstants.CANDLE_ID, Constants.CANIVORE_NAME);

    private static enum LEDGroup {
        TOP(LEDConstants.LED_COUNT * (1 / 4), LEDConstants.LED_COUNT * (2 / 4)),
        BOTTOM(0, LEDConstants.LED_COUNT * (3 / 4))

        ;
        private final StrobeAnimation m_animationStrobe1 = new StrobeAnimation(255, 0, 0);
        private final StrobeAnimation m_animationStrobe2 = new StrobeAnimation(255, 0, 0);

        private int m_start1;
        private int m_start2;

        LEDGroup(int start1, int start2) {
            m_start1 = start1;
            m_start2 = start2;

            m_animationStrobe1.setLedOffset(m_start1);
            m_animationStrobe1.setNumLed(LEDConstants.GROUP_SIZE);
            m_animationStrobe2.setLedOffset(m_start2);
            m_animationStrobe2.setNumLed(LEDConstants.GROUP_SIZE);
        }

        public void setStatus(LEDColorStatus colorStatus) {
            // System.out.println("RED " + colorStatus.m_red);
            // System.out.println("GREEN " + colorStatus.m_green);
            // System.out.println("BLUE " + colorStatus.m_blue);
            m_animationStrobe1.setR(colorStatus.m_red);
            m_animationStrobe1.setG(colorStatus.m_green);
            m_animationStrobe1.setB(colorStatus.m_blue);

            m_animationStrobe2.setR(colorStatus.m_red);
            m_animationStrobe2.setG(colorStatus.m_green);
            m_animationStrobe2.setB(colorStatus.m_blue);
        }
    }

    private static class LEDColor {
        public int m_red;
        public int m_green;
        public int m_blue;

        LEDColor(int red, int green, int blue) {
            m_red = red;
            m_green = green;
            m_blue = blue;
        }
    }
    // private final LEDColor COLOR_RED = new LEDColor(255, 0, 0);
    // private final LEDColor COLOR_YELLOW = new LEDColor(255, 255, 0);
    // private final LEDColor COLOR_GREEN = new LEDColor(0, 255, 0);

    private static class LEDColorStatus extends LEDColor {
        private int level = 0;
        LEDColorStatus() {
            super(255, 0, 0);
        }

        private void update() {
            switch (level) {
                case 0:
                    m_red = 255;
                    m_green = 0;
                    m_blue = 0;
                    break;
                case 1:
                    m_red = 255;
                    m_green = 255;
                    m_blue = 0;
                    break;
                case 2:
                    m_red = 0;
                    m_green = 255;
                    m_blue = 0;
                    break;
                default:
                    break;
            }
        }

        public LEDColorStatus up() {
            if (level == 2)
                // throw new Exception("led color status level too high");
                return this;

            level++;
            update();

            return this;
        }
        // public LEDColorStatus down() {
        //     if (level == 0)
        //         // throw new Exception("led color status level too low");
        //         return this;

        //     level--;
        //     update();

        //     return this;
        // }
    }

    public LEDSubsystem() {
        final CANdleConfiguration config = new CANdleConfiguration();

        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;

        OurUtils.tryApplyConfig(m_candle, LEDConstants.CANDLE_ID, config);
    }

    @Override
    public void periodic() {
        final LEDColorStatus topStatus = new LEDColorStatus();
        if (RobotState.getVisionPoseSuccess()) {
            // System.out.println("topStatus up 1");
            topStatus.up();
        }
        if (RobotState.getIsLinedUpWithReefYaw()) {
            // System.out.println("topStatus up 2");
            topStatus.up();
        }
        LEDGroup.TOP.setStatus(topStatus);

        final LEDColorStatus bottomStatus = new LEDColorStatus();
        if (ElevatorSubsystem.getSingleton().getIsAtTargetPosition()) {
            // System.out.println("bottomStatus up 1");
            bottomStatus.up();
        }
        if (GantrySubsystem.getSingleton().getIsAtTargetPosition()) {
            // System.out.println("bottomStatus up 2");
            bottomStatus.up();
        }
        LEDGroup.BOTTOM.setStatus(bottomStatus);

        // m_candle.animate(LEDGroup.TOP.m_animationStrobe, 0);
        // m_candle.animate(LEDGroup.BOTTOM.m_animationStrobe, 2);

        final int w = 255;
        m_candle.animate(null, 0);
        m_candle.animate(null, 2);
        m_candle.setLEDs(topStatus.m_red, topStatus.m_green, topStatus.m_blue, w, 8, LEDConstants.GROUP_SIZE);
        m_candle.setLEDs(bottomStatus.m_red, bottomStatus.m_green, bottomStatus.m_blue, w, 8 + LEDConstants.GROUP_SIZE, LEDConstants.GROUP_SIZE);
        m_candle.setLEDs(bottomStatus.m_red, bottomStatus.m_green, bottomStatus.m_blue, w, 8 + (LEDConstants.GROUP_SIZE * 2), LEDConstants.GROUP_SIZE);
        m_candle.setLEDs(topStatus.m_red, topStatus.m_green, topStatus.m_blue, w, 8 + (LEDConstants.GROUP_SIZE * 3), LEDConstants.GROUP_SIZE);
    }
}