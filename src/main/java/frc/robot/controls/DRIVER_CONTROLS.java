// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

public final class DRIVER_CONTROLS {
    private static final CommandXboxController driverController = new CommandXboxController(ControllerConstants.DRIVER_PORT);

    public static final Trigger seedFieldCentric = driverController.start();
    public static final Trigger localizeToReef = driverController.a();
}
