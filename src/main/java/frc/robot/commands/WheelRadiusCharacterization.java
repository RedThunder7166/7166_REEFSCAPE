// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class WheelRadiusCharacterization extends Command {
  private static final double driveRadius = DriveConstants.DRIVE_RADIUS;
  // private static final DoubleSupplier gyroYawRadsSupplier =
  //     () -> RobotState.getInstance().getOdometryPose().getRotation().getRadians();

  public enum Direction {
    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int value;

    Direction(int valueIn) {
      value = valueIn;
    }

    public int getValue() {
      return value;
    }
  }

  private final CommandSwerveDrivetrain drive;
  private final Direction omegaDirection;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private final DoubleSupplier gyroYawRadsSupplier;

  private double lastGyroYawRads = 0.0;
  private double accumGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(CommandSwerveDrivetrain drive, Direction omegaDirection) {
    this.drive = drive;
    this.omegaDirection = omegaDirection;
    addRequirements(drive);

    // gyroYawRadsSupplier = () -> drive.getCustomEstimatedPose().getRotation().getRadians();
    gyroYawRadsSupplier = () -> drive.getState().Pose.getRotation().getRadians();
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumGyroYawRads = 0.0;

    startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    // drive.runWheelRadiusCharacterization(
    //     omegaLimiter.calculate(omegaDirection.value * 0.1));
    drive.setControl(new SwerveRequest.RobotCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withRotationalRate(
        omegaLimiter.calculate(omegaDirection.value * 0.35 * DriveConstants.MAX_ANGULAR_RATE)));

    // Get yaw and wheel positions
    accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositions = drive.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    // Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    // Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
    // Logger.recordOutput(
    //     "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
    //     Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
