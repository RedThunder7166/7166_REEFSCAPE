// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OurUtils;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.RobotState;
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
        // TOP_SUBSET(LEDConstants.LED_START_OFFSET, 4, 12 + (LEDConstants.GROUP_SIZE * 3), 4),
        // TOP(12, LEDConstants.GROUP_SIZE, 16 + (LEDConstants.GROUP_SIZE * 3), LEDConstants.GROUP_SIZE),
        // BOTTOM(12 + LEDConstants.GROUP_SIZE, LEDConstants.GROUP_SIZE, 16 + (LEDConstants.GROUP_SIZE * 2), LEDConstants.GROUP_SIZE)

        TOP_SUBSET(LEDConstants.LED_START_OFFSET, 3, 41, 3),
        TOP(11, 5, 36, 5),
        // BOTTOM(16, 10, 26, 10)
        BOTTOM(16, 20)

        ;
        private final ArrayList<Pair<Integer, Integer>> m_list = new ArrayList<>();

        public final LEDColor m_color = new LEDColor(0, 0, 0);

        LEDGroup(int... numbers) {
            for (int i = 0; i < numbers.length; i += 2)
                m_list.add(new Pair<>(numbers[i], numbers[i + 1]));
        }

        public void apply(CANdle candle) {
            for (int i = 0; i < m_list.size(); i++) {
                final Pair<Integer, Integer> pair = m_list.get(i);
                candle.setLEDs(m_color.m_red, m_color.m_green, m_color.m_blue, m_color.m_white, pair.getFirst().intValue(), pair.getSecond().intValue());
            }
        }
    }

    private static class LEDColor {
        public int m_red;
        public int m_green;
        public int m_blue;
        public int m_white;

        LEDColor(int red, int green, int blue) {
            this(red, green, blue, 0);
        }
        LEDColor(int red, int green, int blue, int white) {
            m_red = red;
            m_green = green;
            m_blue = blue;
            m_white = white;
        }

        public void setRGBW(int red, int green, int blue, int white) {
            m_red = red;
            m_green = green;
            m_blue = blue;
            m_white = white;
        }
        public void setRGB(int red, int green, int blue) {
            setRGBW(red, green, blue, 0);
        }

        public LEDColor steal(LEDColor otherColor) {
            setRGBW(otherColor.m_red, otherColor.m_green, otherColor.m_blue, otherColor.m_white);
            return this;
        }

        public LEDColor red() {
            setRGB(LEDConstants.PRIMARY_HUE, LEDConstants.SECONDARY_HUE, LEDConstants.SECONDARY_HUE);

            return this;
        }
        public LEDColor green() {
            setRGB(LEDConstants.SECONDARY_HUE, LEDConstants.PRIMARY_HUE, LEDConstants.SECONDARY_HUE);

            return this;
        }
        public LEDColor blue() {
            setRGB(LEDConstants.SECONDARY_HUE, LEDConstants.SECONDARY_HUE, LEDConstants.PRIMARY_HUE);

            return this;
        }
        public LEDColor yellow() {
            setRGB(LEDConstants.PRIMARY_HUE, LEDConstants.PRIMARY_HUE, LEDConstants.SECONDARY_HUE);

            return this;
        }

        public LEDColor left() {
            // setRGB(198, 175, 141);
            setRGB(LEDConstants.SECONDARY_HUE, LEDConstants.SECONDARY_HUE, LEDConstants.PRIMARY_HUE);

            return this;
        }

        public LEDColor right() {
            // setRGB(181, 206, 112);
            setRGB(LEDConstants.PRIMARY_HUE, LEDConstants.SECONDARY_HUE, LEDConstants.PRIMARY_HUE);

            return this;
        }
    }
    // private final LEDColor COLOR_RED = new LEDColor(LEDConstants.PRIMARY_HUE, LEDConstants.SECONDARY_HUE, LEDConstants.SECONDARY_HUE);
    // private final LEDColor COLOR_YELLOW = new LEDColor(LEDConstants.PRIMARY_HUE, LEDConstants.PRIMARY_HUE, LEDConstants.SECONDARY_HUE);
    // private final LEDColor COLOR_GREEN = new LEDColor(LEDConstants.SECONDARY_HUE, LEDConstants.PRIMARY_HUE, LEDConstants.SECONDARY_HUE);

    public LEDSubsystem() {
        m_candle.configFactoryDefault();

        m_candle.configLEDType(LEDStripType.GRB);
        m_candle.configBrightnessScalar(LEDConstants.BRIGHTNESS);

        for (int i = 0; i < m_candle.getMaxSimultaneousAnimationCount(); i++)
            m_candle.clearAnimation(i);

        // RobotState.addTelemetry(this::updateAnimation, 20);
    }

    @Override
    public void periodic() {
        if (DriverStation.isEnabled()) {
            // TODO: make sure it can see closest section id
            final boolean visionSuccess = RobotState.getVisionPoseSuccess();
            // final double reefSectionError = RobotState.getReefSectionDegreeError().orElse(0d);

            boolean topSubsetChanged = false;
            if (visionSuccess && RobotState.getCanSeeClosestReefTag()) {
                if (RobotState.getReefTargetForwardDistance().orElse(1000d) <= AprilTagConstants.REEF_FORWARD_LED_THRESHOLD) {
                    LEDGroup.TOP.m_color.green();
                    final double distance = RobotState.getReefTargetHorizontalDistance().orElse(1000d).doubleValue();
                    // if (reefSectionError < -Constants.REEF_YAW_LINEUP_THRESHOLD_DEGREES) {
                    //     topSubsetChanged = true;
                    //     LEDGroup.TOP_SUBSET.m_color.right();
                    // } else if (reefSectionError > Constants.REEF_YAW_LINEUP_THRESHOLD_DEGREES) {
                    //     topSubsetChanged = true;
                    //     LEDGroup.TOP_SUBSET.m_color.left();
                    // }

                    // if (!topSubsetChanged)
                        if (distance < -AprilTagConstants.REEF_HORIZONTAL_LED_THRESHOLD) {
                            topSubsetChanged = true;
                            LEDGroup.TOP_SUBSET.m_color.right();
                        } else if (distance > AprilTagConstants.REEF_HORIZONTAL_LED_THRESHOLD) {
                            topSubsetChanged = true;
                            LEDGroup.TOP_SUBSET.m_color.left();
                        }
                } else
                    LEDGroup.TOP.m_color.yellow();
            } else
                LEDGroup.TOP.m_color.red();

            if (!topSubsetChanged)
                LEDGroup.TOP_SUBSET.m_color.steal(LEDGroup.TOP.m_color);

            final boolean elevatorInPosition = ElevatorSubsystem.getSingleton().getIsAtTargetPosition();
            final boolean gantryInPosition = GantrySubsystem.getSingleton().getIsAtTargetPosition() && !RobotState.getIsGantryAutoAdjustOutOfBounds();

            if (elevatorInPosition == gantryInPosition)
                if (elevatorInPosition)
                    LEDGroup.BOTTOM.m_color.green();
                else
                    LEDGroup.BOTTOM.m_color.red();
            else
                LEDGroup.BOTTOM.m_color.yellow();

        //     for (LEDGroup value : LEDGroup.values())
        //         value.apply(m_candle);
        // } else {
        //     for (int i = LEDConstants.LED_START_OFFSET; i < LEDConstants.LED_END; i++) {
        //         final boolean isChosenOne = i == animationTargetIndex;
        //         final int secondary = isChosenOne ? 0 : 255 - (int) OurUtils.map(i, LEDConstants.LED_START_OFFSET, animationTargetIndex, 0, 255);
        //         m_candle.setLEDs(255, secondary, secondary, 255, i, 1);
        //     }
        } else {
            LEDGroup.TOP.m_color.setRGB(255, 0, 0);
            LEDGroup.TOP_SUBSET.m_color.steal(LEDGroup.TOP.m_color);
            LEDGroup.BOTTOM.m_color.setRGBW(255 / 2, 255 / 2, 255 / 2, 255 / 2);
        }

        for (LEDGroup value : LEDGroup.values())
            value.apply(m_candle);
    }

    private int animationTargetIndex = LEDConstants.LED_START_OFFSET;
    private boolean animationTargetForward = false;

    private void updateAnimation() {
        if (animationTargetForward)
            animationTargetIndex++;
        else
            animationTargetIndex--;

        if (animationTargetIndex < LEDConstants.LED_START_OFFSET) {
            animationTargetIndex = LEDConstants.LED_START_OFFSET;
            animationTargetForward = true;
        } else if (animationTargetIndex > LEDConstants.LED_END) {
            animationTargetIndex = LEDConstants.LED_END;
            animationTargetForward = false;
        }

        SmartDashboard.putNumber("helloo", animationTargetIndex);
    }
}