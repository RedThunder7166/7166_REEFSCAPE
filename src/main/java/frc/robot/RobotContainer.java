// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.DESIRED_CONTROL_TYPE;
import frc.robot.RobotState.RELATIVE_SCORE_POSITION;
import frc.robot.controls.DRIVER_CONTROLS;
import frc.robot.controls.OPERATOR_CONTROLS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GantrySubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorManualDirection;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorState;
import frc.robot.subsystems.GantrySubsystem.GantryManualDirection;
import frc.robot.subsystems.GantrySubsystem.GantryState;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    // private final SwerveRequest.RobotCentric forwardStraightRequest = new SwerveRequest.RobotCentric()
    //         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_joystick = new CommandXboxController(0);

    // private final CommandSwerveDrivetrain m_swerveSubsystem = TunerConstants.createDrivetrain();

    private final CameraSubsystem m_cameraSubsystem = CameraSubsystem.getSingleton();
    private final ElevatorSubsystem m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
    // private final GantrySubsystem m_gantrySubsystem = GantrySubsystem.getSingleton();
    // private final IntakeOuttakeSubsystem m_intakeOuttakeSubsystem = IntakeOuttakeSubsystem.getSingleton();

    private InstantCommand makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION desiredPosition, ElevatorState desiredElevatorState, GantryState desiredGantryState) {
        // return new InstantCommand(() -> {
        //     RobotState.setTargetScorePosition(desiredPosition);
        //     m_elevatorSubsystem.setAutomaticState(desiredElevatorState);
        //     m_gantrySubsystem.setAutomaticState(desiredGantryState);
        // }, m_elevatorSubsystem, m_gantrySubsystem);
        return new InstantCommand(() -> {
            RobotState.setTargetScorePosition(desiredPosition);
            m_elevatorSubsystem.setDesiredControlType(DESIRED_CONTROL_TYPE.AUTOMATIC);

            m_elevatorSubsystem.setAutomaticState(desiredElevatorState);
        }, m_elevatorSubsystem);
    }
    public final InstantCommand positionCoralStation;
    public final InstantCommand setTargetScorePosition_NONE;

    public final InstantCommand setTargetScorePosition_L1;
    public final InstantCommand setTargetScorePosition_L2_L;
    public final InstantCommand setTargetScorePosition_L2_R;
    public final InstantCommand setTargetScorePosition_L3_L;
    public final InstantCommand setTargetScorePosition_L3_R;
    public final InstantCommand setTargetScorePosition_L4_L;
    public final InstantCommand setTargetScorePosition_L4_R;

    // private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        // m_autoChooser = AutoBuilder.buildAutoChooser("thereisnoauto");
        // SmartDashboard.putData("AutoChooser", m_autoChooser);

        positionCoralStation = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.NONE, ElevatorState.CORAL_STATION, GantryState.LOADING);
        setTargetScorePosition_NONE = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.NONE, ElevatorState.IDLE, GantryState.IDLE);

        setTargetScorePosition_L1 = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L1, ElevatorState.SCORE, GantryState.SCORE);
        setTargetScorePosition_L2_L = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L2_L, ElevatorState.SCORE, GantryState.SCORE);
        setTargetScorePosition_L2_R = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L2_R, ElevatorState.SCORE, GantryState.SCORE);
        setTargetScorePosition_L3_L = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L3_L, ElevatorState.SCORE, GantryState.SCORE);
        setTargetScorePosition_L3_R = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L3_R, ElevatorState.SCORE, GantryState.SCORE);
        setTargetScorePosition_L4_L = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L4_L, ElevatorState.SCORE, GantryState.SCORE);
        setTargetScorePosition_L4_R = makeSetTargetScorePositionCommand(RELATIVE_SCORE_POSITION.L4_R, ElevatorState.SCORE, GantryState.SCORE);

        configureBindings();
    }

    private void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // m_swerveSubsystem.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     m_swerveSubsystem.applyRequest(() ->
        //         driveRequest.withVelocityX(-m_joystick.getLeftY() * MaxSpeed)
        //             .withVelocityY(-m_joystick.getLeftX() * MaxSpeed)
        //             .withRotationalRate(-m_joystick.getRightX() * MaxAngularRate)
        //     )
        // );

        // joystick.a().whileTrue(swerveSubsystem.applyRequest(() -> brakeRequest));
        // joystick.b().whileTrue(swerveSubsystem.applyRequest(() ->
        //     pointRequest.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // joystick.pov(0).whileTrue(swerveSubsystem.applyRequest(() ->
        //     forwardStraightRequest.withVelocityX(0.5).withVelocityY(0))
        // );
        // joystick.pov(180).whileTrue(swerveSubsystem.applyRequest(() ->
        //     forwardStraightRequest.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(swerveSubsystem.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(swerveSubsystem.sysIdQuasistatic(Direction.kReverse));

        // DRIVER CONTROLS

        // reset the field-centric heading
        // DRIVER_CONTROLS.seedFieldCentric.onTrue(m_swerveSubsystem.runOnce(() -> m_swerveSubsystem.seedFieldCentric()));
        DRIVER_CONTROLS.localizeToReef.whileTrue(m_cameraSubsystem.localizeToReefCommand);

        // OPERATOR CONTROLS

        // OPERATOR_CONTROLS.INTAKE_OUT.whileTrue(m_intakeOuttakeSubsystem.m_outCommand);
        // OPERATOR_CONTROLS.INTAKE_IN.whileTrue(m_intakeOuttakeSubsystem.m_inCommand);

        OPERATOR_CONTROLS.POSITION_CORAL_STATION.onTrue(positionCoralStation);

        OPERATOR_CONTROLS.SCORE_L1.onTrue(setTargetScorePosition_L1);
        OPERATOR_CONTROLS.SCORE_L2_L.onTrue(setTargetScorePosition_L2_L);
        OPERATOR_CONTROLS.SCORE_L2_R.onTrue(setTargetScorePosition_L2_R);
        OPERATOR_CONTROLS.SCORE_L3_L.onTrue(setTargetScorePosition_L3_L);
        OPERATOR_CONTROLS.SCORE_L3_R.onTrue(setTargetScorePosition_L3_R);
        OPERATOR_CONTROLS.SCORE_L4_L.onTrue(setTargetScorePosition_L4_L);
        OPERATOR_CONTROLS.SCORE_L4_R.onTrue(setTargetScorePosition_L4_R);

        OPERATOR_CONTROLS.ELEVATOR_MANUAL_UP.whileTrue(m_elevatorSubsystem.m_manualUpCommand);
        OPERATOR_CONTROLS.ELEVATOR_MANUAL_DOWN.whileTrue(m_elevatorSubsystem.m_manualDownCommand);

        // OPERATOR_CONTROLS.GANTRY_MANUAL_LEFT.whileTrue(m_gantrySubsystem.m_manualLeftCommand);
        // OPERATOR_CONTROLS.GANTRY_MANUAL_RIGHT.whileTrue(m_gantrySubsystem.m_manualRightCommand);

        // m_swerveSubsystem.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // return m_autoChooser.getSelected();
        return Commands.none();
        // return m_elevatorSubsystem.getTempGoUntilTargetIncreaseCommand(2.54).andThen(m_elevatorSubsystem.getTempHoldPositionCommand());
    }

    public void teleopInit() {
        RobotState.setTargetScorePosition(RELATIVE_SCORE_POSITION.NONE);

        m_elevatorSubsystem.resetManualPosition();
        // m_elevatorSubsystem.resetMotorPositions();
    }
}
