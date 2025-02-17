// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState.RelativeScorePosition;
import frc.robot.commands.AutomaticCommands;
import frc.robot.controls.DRIVER_CONTROLS;
import frc.robot.controls.OPERATOR_CONTROLS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CameraSubsystem.RelativeReefLocation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GantrySubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.IntakeOuttakeSubsystemInterface;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric m_robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsystem;

    private final ElevatorSubsystemInterface m_elevatorSubsystem;
    private final GantrySubsystemInterface m_gantrySubsystem;
    private final IntakeOuttakeSubsystemInterface m_intakeOuttakeSubsystem;

    private final Rotation2d m_initialSwerveRotation;

    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_driveSubsystem = TunerConstants.createDrivetrain();
        m_cameraSubsystem = CameraSubsystem.getSingleton();
        m_cameraSubsystem.setDriveSubsystem(m_driveSubsystem, MaxSpeed, MaxAngularRate);

        m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        m_gantrySubsystem = GantrySubsystem.getSingleton();
        m_intakeOuttakeSubsystem = IntakeOuttakeSubsystem.getSingleton();

        m_autoChooser = AutoBuilder.buildAutoChooser("thereisnoauto");
        SmartDashboard.putData("AutoChooser", m_autoChooser);

        // make sure forward faces red alliance wall
        if (Constants.ALLIANCE == Alliance.Red)
            m_initialSwerveRotation = Rotation2d.kZero;
        else
            m_initialSwerveRotation = Rotation2d.k180deg;

        m_driveSubsystem.ensureThisFileHasBeenModified();

        m_driveSubsystem.resetCustomEstimatedRotation(m_initialSwerveRotation);

        NamedCommands.registerCommand("PositionNone", AutomaticCommands.positionNONE);
        NamedCommands.registerCommand("PositionCoralStation", AutomaticCommands.positionCoralStation);

        NamedCommands.registerCommand("PositionL1", AutomaticCommands.position_L1);
        NamedCommands.registerCommand("PositionL2_L", AutomaticCommands.position_L2_L);
        NamedCommands.registerCommand("PositionL2_R", AutomaticCommands.position_L2_R);
        NamedCommands.registerCommand("PositionL3_L", AutomaticCommands.position_L3_L);
        NamedCommands.registerCommand("PositionL3_R", AutomaticCommands.position_L3_R);
        NamedCommands.registerCommand("PositionL4_L", AutomaticCommands.position_L4_L);
        NamedCommands.registerCommand("PositionL4_R", AutomaticCommands.position_L4_R);

        configureBindings();
    }

    private boolean m_robotCentricForward = false;
    private boolean m_robotCentricRight = false;
    private boolean m_robotCentricBackward = false;
    private boolean m_robotCentricLeft = false;
    private static final double robotCentricSpeed = 0.6;

    private RelativeReefLocation m_targetReefLocation = RelativeReefLocation.AB;
    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        m_driveSubsystem.setDefaultCommand(
            m_driveSubsystem.applyRequest(() -> {
                    if (m_robotCentricForward || m_robotCentricRight || m_robotCentricBackward || m_robotCentricLeft) {
                        double x = 0;
                        double y = 0;
                        if (m_robotCentricForward) x = robotCentricSpeed;
                        if (m_robotCentricRight) y = -robotCentricSpeed;
                        if (m_robotCentricBackward) x = -robotCentricSpeed;
                        if (m_robotCentricLeft) y = robotCentricSpeed;
                        return m_robotCentricRequest.withVelocityX(x).withVelocityY(y)
                            .withRotationalRate(-DRIVER_CONTROLS.getRightX() * MaxAngularRate);
                    } else
                        return m_fieldCentricRequest.withVelocityX(-DRIVER_CONTROLS.getLeftY() * MaxSpeed)
                            .withVelocityY(-DRIVER_CONTROLS.getLeftX() * MaxSpeed)
                            .withRotationalRate(-DRIVER_CONTROLS.getRightX() * MaxAngularRate);
                }
            )
        );

        // DRIVER CONTROLS

        DRIVER_CONTROLS.brake.whileTrue(m_driveSubsystem.applyRequest(() -> m_brakeRequest));
        DRIVER_CONTROLS.seedFieldCentric.onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.seedFieldCentric()));
        // reset rotation to set rotation based on alliance
        DRIVER_CONTROLS.seedRotation.onTrue(m_driveSubsystem.runOnce(() -> {
            m_driveSubsystem.resetCustomEstimatedRotation(m_initialSwerveRotation);
        }));

        DRIVER_CONTROLS.robotCentricForward.whileTrue(Commands.startEnd(() -> {
            m_robotCentricForward = true;
        }, () -> {
            m_robotCentricForward = false;
        }));
        DRIVER_CONTROLS.robotCentricRight.whileTrue(Commands.startEnd(() -> {
            m_robotCentricRight = true;
        }, () -> {
            m_robotCentricRight = false;
        }));
        DRIVER_CONTROLS.robotCentricBackward.whileTrue(Commands.startEnd(() -> {
            m_robotCentricBackward = true;
        }, () -> {
            m_robotCentricBackward = false;
        }));
        DRIVER_CONTROLS.robotCentricLeft.whileTrue(Commands.startEnd(() -> {
            m_robotCentricLeft = true;
        }, () -> {
            m_robotCentricLeft = false;
        }));

        SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
        DRIVER_CONTROLS.incrementTargetReefLocation.onTrue(new InstantCommand(() -> {
            m_targetReefLocation = m_targetReefLocation.getNext();
            SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
        }));
        DRIVER_CONTROLS.decrementTargetReefLocation.onTrue(new InstantCommand(() -> {
            m_targetReefLocation = m_targetReefLocation.getPrevious();
            SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
        }));

        DRIVER_CONTROLS.localizeToReef.whileTrue(new CameraSubsystem.DynamicCommand(() -> {
            return m_cameraSubsystem.getPathCommandFromReefTag(m_targetReefLocation);
        }));

        DRIVER_CONTROLS.TEMPORARY_resetGantryPosition.onTrue(m_gantrySubsystem.addToCommandRequirements(new InstantCommand(() -> {
            m_gantrySubsystem.resetMotorPosition();
            m_gantrySubsystem.resetManualPosition();
        })));

        // OPERATOR CONTROLS

        OPERATOR_CONTROLS.INTAKE_OUT.whileTrue(m_intakeOuttakeSubsystem.getOutCommand());
        OPERATOR_CONTROLS.INTAKE_IN.whileTrue(m_intakeOuttakeSubsystem.getInCommand());

        OPERATOR_CONTROLS.POSITION_CORAL_STATION.onTrue(AutomaticCommands.positionCoralStation);

        OPERATOR_CONTROLS.SCORE_L1.onTrue(AutomaticCommands.position_L1);
        OPERATOR_CONTROLS.SCORE_L2_L.onTrue(AutomaticCommands.position_L2_L);
        OPERATOR_CONTROLS.SCORE_L2_R.onTrue(AutomaticCommands.position_L2_R);
        OPERATOR_CONTROLS.SCORE_L3_L.onTrue(AutomaticCommands.position_L3_L);
        OPERATOR_CONTROLS.SCORE_L3_R.onTrue(AutomaticCommands.position_L3_R);
        OPERATOR_CONTROLS.SCORE_L4_L.onTrue(AutomaticCommands.position_L4_L);
        OPERATOR_CONTROLS.SCORE_L4_R.onTrue(AutomaticCommands.position_L4_R);

        OPERATOR_CONTROLS.ELEVATOR_MANUAL_UP.whileTrue(m_elevatorSubsystem.getManualUpCommand());
        OPERATOR_CONTROLS.ELEVATOR_MANUAL_DOWN.whileTrue(m_elevatorSubsystem.getManualDownCommand());

        OPERATOR_CONTROLS.GANTRY_MANUAL_LEFT.whileTrue(m_gantrySubsystem.getManualLeftCommand());
        OPERATOR_CONTROLS.GANTRY_MANUAL_RIGHT.whileTrue(m_gantrySubsystem.getManualRightCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
        // return m_elevatorSubsystem.getTempGoUntilTargetIncreaseCommand(2.54).andThen(m_elevatorSubsystem.getTempHoldPositionCommand());
        // return m_elevatorSubsystem.getTimeTravelCommand(4.2);
    }

    public void teleopInit() {
        RobotState.setTargetScorePosition(RelativeScorePosition.NONE);

        m_elevatorSubsystem.resetManualPosition();
    }
}
