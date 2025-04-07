// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState.ClimbActuatorState;
import frc.robot.RobotState.GamePieceType;
import frc.robot.RobotState.IntakeState;
import frc.robot.RobotState.TargetScorePosition;
import frc.robot.commands.AutomaticCommands;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.controls.DRIVER_CONTROLS;
import frc.robot.controls.OPERATOR_CONTROLS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeHandSubsystem;
import frc.robot.subsystems.AlgaeMouthSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CameraSubsystem.CoralStationID;
import frc.robot.subsystems.CameraSubsystem.RelativeReefLocation;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GantrySubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.AlgaeHandSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.AlgaeMouthSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.ClimbSubsystemInterface;
import frc.robot.subsystems.SubsystemInterfaces.IntakeOuttakeSubsystemInterface;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * 0.1).withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric m_robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final CommandSwerveDrivetrain m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsystem;

    private final ElevatorSubsystemInterface m_elevatorSubsystem;
    private final GantrySubsystemInterface m_gantrySubsystem;
    private final IntakeOuttakeSubsystemInterface m_intakeOuttakeSubsystem;
    private final ClimbSubsystemInterface m_climbSubsystem;

    private static final boolean useAlgaeMouth = false;
    private AlgaeHandSubsystemInterface m_algaeHandSubsystem;
    private AlgaeMouthSubsystemInterface m_algaeMouthSubsystem;

    private RelativeReefLocation m_closestReefLocation;

    // NOTE: we use dynamic command to account for alliancestation changing
    private Command createLocalizeToReefCommand() {
        return new CameraSubsystem.DynamicCommand(() -> m_cameraSubsystem.getPathCommandFromReefTag(m_closestReefLocation, false));
    }
    private Command createLocalizeToReefCommand(RelativeReefLocation targetReefLocation, boolean forAuto) {
        return new CameraSubsystem.DynamicCommand(() -> m_cameraSubsystem.getPathCommandFromReefTag(targetReefLocation, forAuto));
    }
    private Command createLocalizeToCoralStationCommand(CoralStationID targetCoralStation) {
        return new CameraSubsystem.DynamicCommand(() -> m_cameraSubsystem.getPathCommandFromCoralStationTag(targetCoralStation));
    }

    private Command createScoreCoralCommand() {
        return m_intakeOuttakeSubsystem.addToCommandRequirements(
            Commands.waitUntil(() -> DriverStation.isTeleop() || m_gantrySubsystem.getIsAtTargetPosition())
            .withTimeout(0.5)
            .andThen(
                Commands.startEnd(
                    () -> RobotState.setWantsToScoreCoral(true),
                    () -> RobotState.setWantsToScoreCoral(false)
                )
                .until(() -> !m_gantrySubsystem.getScoreExitSensorTripped())
            )
        ).andThen(Commands.waitSeconds(AutoConstants.TIME_UNTIL_CORAL_IS_SCORED_SECONDS));
    }
    private Command createScoreAlgaeCommand() {
        return Commands.startEnd(
            () -> RobotState.setWantsToScoreAlgae(true),
            () -> RobotState.setWantsToScoreAlgae(false)
        );
    }

    private Command createPickupCoralCommand() {
        return m_intakeOuttakeSubsystem.addToCommandRequirements(
            Commands.startEnd(
                () -> RobotState.startIntake(IntakeState.IN),
                () -> RobotState.stopIntake()
            )
            // .until(() -> RobotState.getCoralIsGood())
        );
    }

    private Command createAutoPickupCoralCommand() {
        // FIXME: this is broken?; path planner doesn't let us have a parallel group with a path plan command and this
        // return m_intakeOuttakeSubsystem.addToCommandRequirements(
        //     new AutoPickupCommand()
        // );
        // return Commands.runOnce(() -> RobotState.startIntake(IntakeState.IN));
        return Commands.startEnd(
            () -> RobotState.startIntake(IntakeState.IN),
            () -> RobotState.stopIntake()
        ).until(() -> m_gantrySubsystem.getScoreEnterSensorTripped());
    }

    private Command createSetClimbStateCommand(ClimbActuatorState climbState) {
        return m_climbSubsystem.addToCommandRequirements(Commands.runOnce(() -> {
            RobotState.setClimbActuatorState(climbState);
        }));
    }

    private final SendableChooser<Command> m_autoChooser;

    private Command getWaitUntilCoralIsGoodCommand() {
        return Commands.waitUntil(RobotState::getCoralIsGood);
    }

    private SequentialCommandGroup createPrepareScoreCommand(TargetScorePosition scorePosition) {
        return getWaitUntilCoralIsGoodCommand()
            .andThen(AutomaticCommands.createInstantGoToPositionCommand(scorePosition.toStage1()))
            .andThen(m_cameraSubsystem.getWaitUntilWithinGoToPositionDistance())
            .andThen(AutomaticCommands.createGoToPositionCommand(scorePosition));
    }
    private Command createDynamicPrepareScoreCommand() {
        return new CameraSubsystem.DynamicCommand(() -> createPrepareScoreCommand(AutomaticCommands.getTargetScorePosition()));
    }

    // NOTE: these assume autonomous!!!
    private SequentialCommandGroup createGenericEntireScoreCommand(Command driveCommand, TargetScorePosition scorePosition) {
        return new SequentialCommandGroup(
            driveCommand
                .deadlineFor(createPrepareScoreCommand(scorePosition)),
            AutomaticCommands.createGoToPositionCommand(scorePosition),
            createScoreCoralCommand()
        );
    }
    private SequentialCommandGroup createEntireScoreCommand(TargetScorePosition scorePosition, RelativeReefLocation reefPosition) {
        return createGenericEntireScoreCommand(createLocalizeToReefCommand(reefPosition, true), scorePosition);
    }
    // private SequentialCommandGroup createEntireScoreCommand(TargetScorePosition scorePosition, PathPlannerPath scorePath) {
    //     return createGenericEntireScoreCommand(AutoBuilder.followPath(scorePath), scorePosition);
    // }
    private SequentialCommandGroup createGenericEntirePickupCommand(Command driveCommand) {
        return new SequentialCommandGroup(
            AutomaticCommands.createInstantGoToPositionCommand(TargetScorePosition.CORAL_STATION),
            driveCommand,
            createAutoPickupCoralCommand()
        );
    }
    private SequentialCommandGroup createEntirePickupCommand(CoralStationID coralStation) {
        return createGenericEntirePickupCommand(createLocalizeToCoralStationCommand(coralStation));
    }
    // private SequentialCommandGroup createEntirePickupCommand(PathPlannerPath coralStationPath) {
    //     return createGenericEntirePickupCommand(AutoBuilder.followPath(coralStationPath));
    // }
    private class AutoCommandSequence extends SequentialCommandGroup {
        public AutoCommandSequence score(TargetScorePosition scorePosition, RelativeReefLocation reefLocation) {
            addCommands(createEntireScoreCommand(scorePosition, reefLocation));
            return this;
        }
        public AutoCommandSequence pickup(CoralStationID coralStation) {
            addCommands(createEntirePickupCommand(coralStation));
            return this;
        }
        public AutoCommandSequence finish() {
            addCommands(AutomaticCommands.createInstantGoToPositionCommand(TargetScorePosition.CORAL_STATION));
            return this;
        }
    }

    private boolean m_followPathCommandIsDone = false;
    private boolean m_pathFindingCommandIsDone = false;

    private void followPathCommandFinish() {
        m_followPathCommandIsDone = true;
        SmartDashboard.putBoolean("PathPlannerWarmedUp", m_followPathCommandIsDone && m_pathFindingCommandIsDone);
    }
    private void pathFindingCommandFinish() {
        m_pathFindingCommandIsDone = true;
        SmartDashboard.putBoolean("PathPlannerWarmedUp", m_followPathCommandIsDone && m_pathFindingCommandIsDone);
    }

    private static final boolean showAllAutos = false;

    public RobotContainer() {
        m_driveSubsystem = TunerConstants.createDrivetrain();
        m_cameraSubsystem = CameraSubsystem.getSingleton();
        m_cameraSubsystem.setDriveSubsystem(m_driveSubsystem, DriveConstants.MAX_SPEED, DriveConstants.MAX_ANGULAR_RATE);

        m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        m_gantrySubsystem = GantrySubsystem.getSingleton();
        m_intakeOuttakeSubsystem = IntakeOuttakeSubsystem.getSingleton();
        m_climbSubsystem = ClimbSubsystem.getSingleton();
        if (useAlgaeMouth)
            m_algaeMouthSubsystem = AlgaeMouthSubsystem.getSingleton();
        else
            m_algaeHandSubsystem = AlgaeHandSubsystem.getSingleton();

        m_driveSubsystem.ensureThisFileHasBeenModified();

        NamedCommands.registerCommand("PositionNone", AutomaticCommands.positionNONE);
        NamedCommands.registerCommand("GoToCoralStation", AutomaticCommands.createInstantGoToPositionCommand(TargetScorePosition.CORAL_STATION));

        for (var value : TargetScorePosition.values())
            NamedCommands.registerCommand("Position" + value.toString(), AutomaticCommands.createSetTargetScorePositionCommand(value));

        NamedCommands.registerCommand("GoToPosition", AutomaticCommands.createGoToPositionCommand());

        NamedCommands.registerCommand("ScoreCoral", createScoreCoralCommand());
        NamedCommands.registerCommand("PickupCoral", createAutoPickupCoralCommand());

        NamedCommands.registerCommand("StartIntake", m_intakeOuttakeSubsystem.addToCommandRequirements(Commands.runOnce(() -> RobotState.startIntake(IntakeState.IN))));

        NamedCommands.registerCommand("PrepareScore", createDynamicPrepareScoreCommand());

        try {
            var config = RobotConfig.fromGUISettings();
            m_driveSubsystem.configureAutoBuilder(config);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        for (var value : RelativeReefLocation.values())
            NamedCommands.registerCommand("LocalizeToReef" + value.toString(), createLocalizeToReefCommand(value, true));

        NamedCommands.registerCommand("LocalizeToCoralStationLeft", createLocalizeToCoralStationCommand(CoralStationID.Left));
        NamedCommands.registerCommand("LocalizeToCoralStationRight", createLocalizeToCoralStationCommand(CoralStationID.Right));

        m_autoChooser = showAllAutos ? AutoBuilder.buildAutoChooser()
            : AutoBuilder.buildAutoChooserWithOptionsModifier((stream) -> stream.filter(auto -> auto.getName().startsWith("COMP")));
        m_autoChooser.setDefaultOption("thereisnoauto", Commands.none());
        m_autoChooser.addOption("Drive Wheel Radius Characterization", m_driveSubsystem
            .orientModules(CommandSwerveDrivetrain.getCircleOrientations())
            .andThen(Commands.print("modules oriented."))
            .andThen(new WheelRadiusCharacterization(m_driveSubsystem, Direction.COUNTER_CLOCKWISE)));

        // simple autos
        m_autoChooser.addOption("JustLeave", Commands.startEnd(() ->
            m_driveSubsystem.setControl(m_robotCentricRequest.withVelocityX(DriveConstants.MAX_SPEED * 0.25).withVelocityY(0))
        , () ->
            m_driveSubsystem.setControl(m_robotCentricRequest.withVelocityX(0).withVelocityY(0))
        , m_driveSubsystem).withTimeout(0.5));
        m_autoChooser.addOption("OneReefMiddleL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.GH)
            .finish()
        );

        // left autos
        m_autoChooser.addOption("OneReefLeftL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.IJ)
            .finish()
        );
        m_autoChooser.addOption("TwoReefLeftL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.IJ)
            .pickup(CoralStationID.Left)
            .score(TargetScorePosition.L4_L, RelativeReefLocation.KL)
            .finish()
        );
        m_autoChooser.addOption("ThreeReefLeftL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.IJ)
            .pickup(CoralStationID.Left)
            .score(TargetScorePosition.L4_L, RelativeReefLocation.KL)
            .pickup(CoralStationID.Left)
            .score(TargetScorePosition.L4_R, RelativeReefLocation.KL)
            .finish()
        );
        m_autoChooser.addOption("FourReefLeftL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.IJ)
            .pickup(CoralStationID.Left)
            .score(TargetScorePosition.L4_R, RelativeReefLocation.IJ)
            .pickup(CoralStationID.Left)
            .score(TargetScorePosition.L4_L, RelativeReefLocation.KL)
            .pickup(CoralStationID.Left)
            .score(TargetScorePosition.L4_R, RelativeReefLocation.KL)
            .finish()
        );

        // right autos
        m_autoChooser.addOption("OneReefRightL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.EF)
            .finish()
        );
        m_autoChooser.addOption("TwoReefRightL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.EF)
            .pickup(CoralStationID.Right)
            .score(TargetScorePosition.L4_L, RelativeReefLocation.CD)
            .finish()
        );
        m_autoChooser.addOption("ThreeReefRightL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.EF)
            .pickup(CoralStationID.Right)
            .score(TargetScorePosition.L4_L, RelativeReefLocation.CD)
            .pickup(CoralStationID.Right)
            .score(TargetScorePosition.L4_R, RelativeReefLocation.CD)
            .finish()
        );
        m_autoChooser.addOption("FourReefRightL4Localize", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.EF)
            .pickup(CoralStationID.Right)
            .score(TargetScorePosition.L4_R, RelativeReefLocation.EF)
            .pickup(CoralStationID.Right)
            .score(TargetScorePosition.L4_L, RelativeReefLocation.CD)
            .pickup(CoralStationID.Right)
            .score(TargetScorePosition.L4_R, RelativeReefLocation.CD)
            .finish()
        );

        // test autos
        m_autoChooser.addOption("TESTSpeed1", new AutoCommandSequence()
            .score(TargetScorePosition.L4_L, RelativeReefLocation.KL)
            .pickup(CoralStationID.Left)
            .finish()
        );

        // try {
        //     m_autoChooser.addOption("TESTLocalizeLeft", new SequentialCommandGroup(
        //         createEntireScoreCommand(TargetScorePosition.L4_L, RelativeReefLocation.IJ),
        //         createEntirePickupCommand(PathPlannerPath.fromPathFile("I-JtoCoralStation")),
        //         createEntireScoreCommand(TargetScorePosition.L4_L, PathPlannerPath.fromPathFile("CoralStationToK-L")),
        //         createEntirePickupCommand(PathPlannerPath.fromPathFile("K-LtoCoralStation")),
        //         createEntireScoreCommand(TargetScorePosition.L4_R, PathPlannerPath.fromPathFile("CoralStationToK-L"))
        //     ));
        // } catch (FileVersionException | IOException | ParseException ex) {
        //     DriverStation.reportError("Failed to load read path from file", ex.getStackTrace());
        // }

        SmartDashboard.putData("AutoChooser", m_autoChooser);

        SmartDashboard.putBoolean("PathPlannerWarmedUp", false);
        FollowPathCommand.warmupCommand()
            .andThen(this::followPathCommandFinish)
            .ignoringDisable(true)
            .schedule();
        PathfindingCommand.warmupCommand()
            .andThen(this::pathFindingCommandFinish)
            .ignoringDisable(true)
            .schedule();

        initialize();
    }

    private boolean m_robotCentricForward = false;
    private boolean m_robotCentricRight = false;
    private boolean m_robotCentricBackward = false;
    private boolean m_robotCentricLeft = false;
    private static final double robotCentricSpeed = 0.6;

    private void initialize() {
        DriverStation.silenceJoystickConnectionWarning(true);
        RobotState.updateState(this);

        m_driveSubsystem.setDefaultCommand(
            m_driveSubsystem.applyRequest(() -> {
                    final double rotation = RobotState.getDriveRotationOverride().orElseGet(() -> -DRIVER_CONTROLS.getRightX() * DriveConstants.MAX_ANGULAR_RATE);
                    if (m_robotCentricForward || m_robotCentricRight || m_robotCentricBackward || m_robotCentricLeft) {
                        double x = 0;
                        double y = 0;
                        if (m_robotCentricForward) x = robotCentricSpeed;
                        if (m_robotCentricRight) y = -robotCentricSpeed;
                        if (m_robotCentricBackward) x = -robotCentricSpeed;
                        if (m_robotCentricLeft) y = robotCentricSpeed;
                        return m_robotCentricRequest.withVelocityX(x).withVelocityY(y)
                            .withRotationalRate(rotation);
                    } else
                        return m_fieldCentricRequest.withVelocityX(-DRIVER_CONTROLS.getLeftY() * DriveConstants.MAX_SPEED)
                            .withVelocityY(-DRIVER_CONTROLS.getLeftX() * DriveConstants.MAX_SPEED)
                            .withRotationalRate(rotation);
                }
            )
        );

        // DRIVER CONTROLS

        // DRIVER_CONTROLS.brake.whileTrue(m_driveSubsystem.applyRequest(() -> m_brakeRequest));
        DRIVER_CONTROLS.seedFieldCentric.onTrue(m_driveSubsystem.runOnce(() -> m_driveSubsystem.seedFieldCentric()));
        // reset rotation to set rotation based on alliance
        DRIVER_CONTROLS.seedRotation.onTrue(m_driveSubsystem.runOnce(() -> {
            m_driveSubsystem.resetCustomEstimatedRotation(RobotState.initialSwerveRotation);
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

        DRIVER_CONTROLS.localizeToReefAB.whileTrue(createLocalizeToReefCommand(RelativeReefLocation.AB, false));
        DRIVER_CONTROLS.localizeToReefCD.whileTrue(createLocalizeToReefCommand(RelativeReefLocation.CD, false));
        DRIVER_CONTROLS.localizeToReefEF.whileTrue(createLocalizeToReefCommand(RelativeReefLocation.EF, false));
        DRIVER_CONTROLS.localizeToReefGH.whileTrue(createLocalizeToReefCommand(RelativeReefLocation.GH, false));
        DRIVER_CONTROLS.localizeToReefIJ.whileTrue(createLocalizeToReefCommand(RelativeReefLocation.IJ, false));
        DRIVER_CONTROLS.localizeToReefKL.whileTrue(createLocalizeToReefCommand(RelativeReefLocation.KL, false));

        DRIVER_CONTROLS.localizeToReefClosest.whileTrue(createLocalizeToReefCommand());
        // DRIVER_CONTROLS.localizeToReefClosest.whileTrue(createLocalizeToCoralStationCommand(CoralStationID.Left));

        // OPERATOR CONTROLS

        OPERATOR_CONTROLS.INTAKE_OUT.whileTrue(m_intakeOuttakeSubsystem.addToCommandRequirements(Commands.runEnd(() -> {
            RobotState.startIntake(IntakeState.OUT);
        }, () -> {
            RobotState.stopIntake();
        })));
        OPERATOR_CONTROLS.INTAKE_IN.whileTrue(createPickupCoralCommand());

        OPERATOR_CONTROLS.POSITION_CORAL_STATION.onTrue(AutomaticCommands.createInstantGoToPositionCommand(TargetScorePosition.CORAL_STATION));

        OPERATOR_CONTROLS.SCORE_L1.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L1));
        OPERATOR_CONTROLS.SCORE_L2_L.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L2_L));
        OPERATOR_CONTROLS.SCORE_L2_R.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L2_R));
        OPERATOR_CONTROLS.SCORE_L3_L.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L3_L));
        OPERATOR_CONTROLS.SCORE_L3_R.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L3_R));
        OPERATOR_CONTROLS.SCORE_L4_L.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L4_L));
        OPERATOR_CONTROLS.SCORE_L4_R.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L4_R));

        OPERATOR_CONTROLS.CORAL_SCORE.whileTrue(createScoreCoralCommand());
        if (useAlgaeMouth)
            OPERATOR_CONTROLS.ALGAE_SCORE.whileTrue(createScoreAlgaeCommand());

        OPERATOR_CONTROLS.MODE_ALGAE.whileTrue(Commands.runOnce(() -> RobotState.setTargetGamePieceType(GamePieceType.ALGAE)));
        OPERATOR_CONTROLS.MODE_CORAL.whileTrue(Commands.runOnce(() -> RobotState.setTargetGamePieceType(GamePieceType.CORAL)));

        OPERATOR_CONTROLS.ELEVATOR_MANUAL_UP.whileTrue(m_elevatorSubsystem.getManualUpCommand());
        OPERATOR_CONTROLS.ELEVATOR_MANUAL_DOWN.whileTrue(m_elevatorSubsystem.getManualDownCommand());

        OPERATOR_CONTROLS.GANTRY_MANUAL_LEFT.whileTrue(m_gantrySubsystem.getManualLeftCommand());
        OPERATOR_CONTROLS.GANTRY_MANUAL_RIGHT.whileTrue(m_gantrySubsystem.getManualRightCommand());

        if (useAlgaeMouth) {
            OPERATOR_CONTROLS.ALGAE_MANUAL_OUT.whileTrue(m_algaeMouthSubsystem.getManualArmOutCommand());
            OPERATOR_CONTROLS.ALGAE_MANUAL_IN.whileTrue(m_algaeMouthSubsystem.getManualArmInCommand());
        } else {
            OPERATOR_CONTROLS.ALGAE_MANUAL_OUT.whileTrue(m_algaeHandSubsystem.getExtendedCommand());
            OPERATOR_CONTROLS.ALGAE_MANUAL_IN.whileTrue(m_algaeHandSubsystem.getHomeCommand());
        }

        OPERATOR_CONTROLS.CLIMBER_OUT.whileTrue(m_climbSubsystem.getManualActuatorOutCommand());
        OPERATOR_CONTROLS.CLIMBER_IN.whileTrue(m_climbSubsystem.getManualActuatorInCommand());

        OPERATOR_CONTROLS.CLIMBER_CLIMB.whileTrue(createSetClimbStateCommand(ClimbActuatorState.CLIMB));
        OPERATOR_CONTROLS.CLIMBER_HOME.whileTrue(createSetClimbStateCommand(ClimbActuatorState.HOME));

        OPERATOR_CONTROLS.CAGE_OUT.whileTrue(m_climbSubsystem.getCageOutCommand());
        OPERATOR_CONTROLS.CAGE_IN.whileTrue(m_climbSubsystem.getCageInCommand());

        OPERATOR_CONTROLS.CLIMB_COMBO.whileTrue(createSetClimbStateCommand(ClimbActuatorState.CLIMB)
            .andThen(m_climbSubsystem.getAutomaticCageOutCommand())
        );
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void update(Rotation2d rotation) {
        m_driveSubsystem.resetCustomEstimatedRotation(rotation);
    }

    private Command m_deployIntakeFlap = Commands.startEnd(
        () -> RobotState.startIntake(IntakeState.IN),
        () -> RobotState.stopIntake()
    ).withTimeout(0.1);

    // private boolean m_autoHasSetRotation = false;

    public void autonomousInit() {
        m_gantrySubsystem.resetManualPosition();
        m_gantrySubsystem.resetMotorPosition();

        // if (!m_autoHasSetRotation && RobotState.initialSwerveRotation != null) {
        //     m_autoHasSetRotation = true;
        //     m_driveSubsystem.resetRotation(RobotState.initialSwerveRotation);
        // }

        if (m_deployIntakeFlap.isScheduled())
            m_deployIntakeFlap.cancel();
        m_deployIntakeFlap.schedule();
    }

    public void autonomousExit() {
        RobotState.setTargetScorePosition(TargetScorePosition.NONE);
        m_elevatorSubsystem.setIdle();
        m_gantrySubsystem.setIdle();
    }

    public void teleopInit() {
        RobotState.setTargetScorePosition(TargetScorePosition.NONE);

        m_elevatorSubsystem.setIdle();
        m_gantrySubsystem.setIdle();

        RobotState.stopIntake();

        // FIXME: better solutions for these
        m_elevatorSubsystem.resetManualPosition();
        m_gantrySubsystem.resetManualPosition();
    }

    public void robotPeriodic() {
        final Pose2d robotPose = m_driveSubsystem.getCustomEstimatedPose();
        Pose2d targetPose = null;
        RelativeReefLocation closestLocation = null;
        double closest_distance = 1000;
        for (var location : RelativeReefLocation.values()) {
            var pose = location.getPose();
            if (pose == null)
                continue;
            final double distance = pose.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < closest_distance) {
                closest_distance = distance;
                targetPose = pose;
                closestLocation = location;
            }
        }

        if (targetPose == null) {
            m_closestReefLocation = closestLocation;
            SmartDashboard.putString("CLOSEST_REEF_LOCATION", "null");
        } else {
            m_closestReefLocation = closestLocation;
            SmartDashboard.putString("CLOSEST_REEF_LOCATION", closestLocation.toString());
            final Translation2d targetCentricTranslation = robotPose.getTranslation().minus(targetPose.getTranslation())
                .rotateBy(targetPose.getRotation().unaryMinus());
            final double horizontalDifference = targetCentricTranslation.getY();
            final double forwardDifference = targetCentricTranslation.getX();

            if (m_cameraSubsystem.getCanAutoAdjust()) {
                RobotState.setReefTargetHorizontalDistance(horizontalDifference);
                RobotState.setReefTargetForwardDistance(forwardDifference);
            } else {
                RobotState.clearReefTargetHorizontalDistance();
                RobotState.clearReefTargetForwardDistance();
            }

            SmartDashboard.putNumber("ReefTargetHorizontalDistance", horizontalDifference);
            SmartDashboard.putNumber("ReefTargetFowardDistanceInches", Units.metersToInches(forwardDifference));
        }
    }
}
