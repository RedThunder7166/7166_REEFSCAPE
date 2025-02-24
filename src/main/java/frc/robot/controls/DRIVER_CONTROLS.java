// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

public final class DRIVER_CONTROLS {
    private static final CommandXboxController driverController = new CommandXboxController(ControllerConstants.DRIVER_PORT);

    public static double getRightX() {
        return driverController.getRightX();
    }
    public static double getLeftX() {
        return driverController.getLeftX();
    }
    public static double getLeftY() {
        return driverController.getLeftY();
    }

    public static final Trigger brake = driverController.a();
    public static final Trigger seedFieldCentric = driverController.start();
    public static final Trigger seedRotation = driverController.back();

    public static final Trigger robotCentricForward = driverController.povUp().or(driverController.povUpLeft()).or(driverController.povUpRight());
    public static final Trigger robotCentricRight = driverController.povRight().or(driverController.povUpRight()).or(driverController.povDownRight());
    public static final Trigger robotCentricBackward = driverController.povDown().or(driverController.povDownLeft()).or(driverController.povDownRight());
    public static final Trigger robotCentricLeft = driverController.povLeft().or(driverController.povUpLeft()).or(driverController.povDownLeft());

    public static final Trigger localizeToReef = driverController.b();
    public static final Trigger decrementTargetReefLocation = driverController.leftBumper();
    public static final Trigger incrementTargetReefLocation = driverController.rightBumper();
}
