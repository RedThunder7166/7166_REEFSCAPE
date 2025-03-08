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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class WheelRadiusCharacterization extends Command {
    private static final double driveRadius = DriveConstants.DRIVE_RADIUS;
    // private static final DoubleSupplier gyroYawRadsSupplier =
    // () -> RobotState.getInstance().getOdometryPose().getRotation().getRadians();

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
    // private double startWheelAverage;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(CommandSwerveDrivetrain drive, Direction omegaDirection) {
        this.drive = drive;
        this.omegaDirection = omegaDirection;
        addRequirements(drive);

        // gyroYawRadsSupplier = () -> drive.getCustomEstimatedPose().getRotation().getRadians();
        // gyroYawRadsSupplier = () -> drive.getState().Pose.getRotation().getRadians();
        gyroYawRadsSupplier = () -> drive.getPigeon2().getYaw().getValue().in(Radians);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
        accumGyroYawRads = 0.0;

        startWheelPositions = drive.getWheelRadiusCharacterizationPosition();

        omegaLimiter.reset(0);

        double[] wheelPositions = drive.getWheelRadiusCharacterizationPosition();
        startWheelPositions = wheelPositions;
    }

    @Override
    public void execute() {
    //     // Run drive at velocity
    //     // drive.runWheelRadiusCharacterization(
    //     // omegaLimiter.calculate(omegaDirection.value * 0.1));
    //     drive.setControl(new SwerveRequest.RobotCentric()
    //             .withDriveRequestType(DriveRequestType.OpenLoopVoltage).withRotationalRate(
    //                     omegaLimiter.calculate(omegaDirection.value * 0.35 * DriveConstants.MAX_ANGULAR_RATE)));

        SmartDashboard.putNumber("DRIVE_RADIUS", driveRadius);

        SmartDashboard.putNumber("GYRO_CURRENT_YAW", gyroYawRadsSupplier.getAsDouble());
    //     SmartDashboard.putNumber("GYRO_LAST_YAW", lastGyroYawRads);

    //     // Get yaw and wheel positions
    //     accumGyroYawRads += MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    //     lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    //     double averageWheelPosition = 0.0;
    //     double[] wheelPositions = drive.getWheelRadiusCharacterizationPosition();
    //     for (int i = 0; i < 4; i++) {
    //         averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    //     }
    //     averageWheelPosition /= 4.0;

    //     SmartDashboard.putNumber("AVERAGE_WHEEL_POSITION", averageWheelPosition);

        SmartDashboard.putNumber("WHEEL_POSITION_0", drive.getWheelRadiusCharacterizationPosition()[0]);

    //     currentEffectiveWheelRadius = (accumGyroYawRads * driveRadius) / averageWheelPosition;
    //     SmartDashboard.putNumber("CURRENT_RADIUS", currentEffectiveWheelRadius);
    }

    @Override
    public void end(boolean interrupted) {
        // if (accumGyroYawRads <= Math.PI * 2.0) {
        //     System.out.println("Not enough data for characterization");
        // } else {
        //     SmartDashboard.putString(
        //             "Effective Wheel Radius:",
        //             Units.metersToInches(currentEffectiveWheelRadius)
        //                     + " inches");
        // }

        double gyroDelta = gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads;
        double wheelPositionDelta = 0;
        double[] wheelPositions = drive.getWheelRadiusCharacterizationPosition();
        for (int i = 0; i < 4; i++) {
            double delta = Math.abs(wheelPositions[i] - startWheelPositions[i]);
            SmartDashboard.putNumber("WHEEL_" + i + "_DELTA", delta);
            wheelPositionDelta += delta;
        }

        wheelPositionDelta = wheelPositionDelta / 4d;

        SmartDashboard.putNumber("WHEEL_POSITION_DELTA", wheelPositionDelta);
        SmartDashboard.putNumber("GYRO_DELTA", gyroDelta);

        SmartDashboard.putString("Effective Wheel Radius:", Units.metersToInches(gyroDelta * driveRadius / wheelPositionDelta) + " inches");
    }
}
