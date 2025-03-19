// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.io.IOException;
import java.util.ArrayList;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState.ClimbActuatorState;
import frc.robot.RobotState.IntakeState;
import frc.robot.RobotState.TargetScorePosition;
import frc.robot.commands.AutomaticCommands;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.commands.WheelRadiusCharacterization.Direction;
import frc.robot.controls.DRIVER_CONTROLS;
import frc.robot.controls.OPERATOR_CONTROLS;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeHandSubsystem;
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

    // private static final boolean shouldRunWheelRadiusCharacterization = true;

    private final CommandSwerveDrivetrain m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsystem;

    private final ElevatorSubsystemInterface m_elevatorSubsystem;
    private final GantrySubsystemInterface m_gantrySubsystem;
    private final IntakeOuttakeSubsystemInterface m_intakeOuttakeSubsystem;
    private final ClimbSubsystemInterface m_climbSubsystem;
    private final AlgaeHandSubsystemInterface m_algaeHandSubsystem;

    // private RelativeReefLocation m_targetReefLocation;
    private RelativeReefLocation m_closestReefLocation;

    // NOTE: we use dynamic command to account for alliancestation changing
    private Command createLocalizeToReefCommand() {
        return new CameraSubsystem.DynamicCommand(() -> m_cameraSubsystem.getPathCommandFromReefTag(m_closestReefLocation, false));
    }
    private Command createLocalizeToReefCommand(RelativeReefLocation targetReefLocation, boolean forAuto) {
        // m_targetReefLocation = targetReefLocation;
        return new CameraSubsystem.DynamicCommand(() -> m_cameraSubsystem.getPathCommandFromReefTag(targetReefLocation, forAuto));
    }
    private Command createLocalizeToCoralStationCommand(CoralStationID targetCoralStation) {
        return new CameraSubsystem.DynamicCommand(() -> m_cameraSubsystem.getPathCommandFromCoralStationTag(targetCoralStation));
    }

    private Command createScoreCommand() {
        return m_intakeOuttakeSubsystem.addToCommandRequirements(new Command() {
            @Override
            public void execute() {
                if (DriverStation.isTeleop() || m_gantrySubsystem.getIsAtTargetPosition())
                    RobotState.setWantsToScore(true);
            }

            @Override
            public boolean isFinished() {
                return !m_gantrySubsystem.getScoreExitSensorTripped();
            }

            @Override
            public void end(boolean isInterrupted) {
                RobotState.setWantsToScore(false);
            }
        }).andThen(Commands.waitSeconds(Constants.TIME_UNTIL_CORAL_IS_SCORED_SECONDS));
    }
    private class PickupCommand extends Command {
        private final GantrySubsystemInterface m_gantrySubsytem;

        public PickupCommand(GantrySubsystemInterface gantrySubsystem) {
            m_gantrySubsytem = gantrySubsystem;
        }

        @Override
        public void execute() {
            if (DriverStation.isAutonomous() || m_gantrySubsystem.getIsAtTargetPosition())
                RobotState.startIntake(IntakeState.IN);
        }

        @Override
        public boolean isFinished() {
            return RobotState.getCoralIsGood();
        }

        @Override
        public void end(boolean isInterrupted) {
            RobotState.stopIntake();
        }
    }
    private final class AutoPickupCommand extends PickupCommand {
        public AutoPickupCommand(GantrySubsystemInterface gantrySubsystem) {
            super(gantrySubsystem);
        }

        @Override
        public boolean isFinished() {
            return m_gantrySubsystem.getScoreEnterSensorTripped();
        }

        @Override
        public void end(boolean isInterrupted) { 
            // do nothing: we don't want the intake to stop here; it will be stopped from auto
        }
    }
    private Command createPickupCommand() {
        return m_intakeOuttakeSubsystem.addToCommandRequirements(
            new PickupCommand(m_gantrySubsystem)
        );
    }

    private Command TESTcreatePickupCommand() {
        return m_intakeOuttakeSubsystem.addToCommandRequirements(
            new AutoPickupCommand(m_gantrySubsystem)
        );
    }

    private Command createSetClimbStateCommand(ClimbActuatorState climbState) {
        return m_climbSubsystem.addToCommandRequirements(new InstantCommand(() -> {
            RobotState.setClimbActuatorState(climbState);
        }));
    }

    private final SendableChooser<Command> m_autoChooser;

    private Command getWaitUntilCoralIsGoodCommand() {
        return Commands.waitUntil(RobotState::getCoralIsGood);
    }

    private SequentialCommandGroup createPrepareScoreCommand(TargetScorePosition scorePosition) {
        return getWaitUntilCoralIsGoodCommand()
            .andThen(AutomaticCommands.createInstantGoToPositionCommand(scorePosition.toL2()))
            .andThen(m_cameraSubsystem.getWaitUntilWithinAutoTargetReefDistanceCommand())
            .andThen(AutomaticCommands.createGoToPositionCommand(scorePosition));
    }
    private Command createDynamicPrepareScoreCommand() {
        return new CameraSubsystem.DynamicCommand(() -> createPrepareScoreCommand(AutomaticCommands.getTargetScorePosition()));
    }

    // NOTE: these assume autonomous!!!
    private SequentialCommandGroup createGenericEntireScoreCommand(Command driveCommand, TargetScorePosition scorePosition) {
        return new SequentialCommandGroup(
            driveCommand
                // toL2 MAY RETURN NULL!
                .deadlineFor(createPrepareScoreCommand(scorePosition)),
            AutomaticCommands.createGoToPositionCommand(scorePosition),
            createScoreCommand()
        );
    }
    private SequentialCommandGroup createEntireScoreCommand(TargetScorePosition scorePosition, RelativeReefLocation reefPosition) {
        return createGenericEntireScoreCommand(createLocalizeToReefCommand(reefPosition, true), scorePosition);
    }
    private SequentialCommandGroup createEntireScoreCommand(TargetScorePosition scorePosition, PathPlannerPath scorePath) {
        return createGenericEntireScoreCommand(AutoBuilder.followPath(scorePath), scorePosition);
    }
    private SequentialCommandGroup createGenericEntirePickupCommand(Command driveCommand) {
        return new SequentialCommandGroup(
            AutomaticCommands.createInstantGoToPositionCommand(TargetScorePosition.CORAL_STATION),
            driveCommand,
            // createPickupCommand()
            TESTcreatePickupCommand()
        );
    }
    private SequentialCommandGroup createEntirePickupCommand(CoralStationID coralStation) {
        return createGenericEntirePickupCommand(createLocalizeToCoralStationCommand(coralStation));
    }
    private SequentialCommandGroup createEntirePickupCommand(PathPlannerPath coralStationPath) {
        return createGenericEntirePickupCommand(AutoBuilder.followPath(coralStationPath));
    }
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

    public RobotContainer() {
        m_driveSubsystem = TunerConstants.createDrivetrain();
        m_cameraSubsystem = CameraSubsystem.getSingleton();
        m_cameraSubsystem.setDriveSubsystem(m_driveSubsystem, DriveConstants.MAX_SPEED, DriveConstants.MAX_ANGULAR_RATE);

        m_elevatorSubsystem = ElevatorSubsystem.getSingleton();
        m_gantrySubsystem = GantrySubsystem.getSingleton();
        m_intakeOuttakeSubsystem = IntakeOuttakeSubsystem.getSingleton();
        m_climbSubsystem = ClimbSubsystem.getSingleton();
        m_algaeHandSubsystem = AlgaeHandSubsystem.getSingleton();

        m_driveSubsystem.ensureThisFileHasBeenModified();

        NamedCommands.registerCommand("PositionNone", AutomaticCommands.positionNONE);
        NamedCommands.registerCommand("GoToCoralStation", AutomaticCommands.createInstantGoToPositionCommand(TargetScorePosition.CORAL_STATION));

        for (var value : TargetScorePosition.values()) {
            NamedCommands.registerCommand("Position" + value.toString(), AutomaticCommands.createSetTargetScorePositionCommand(value));
        }

        // NamedCommands.registerCommand("PositionL1", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L1));
        // NamedCommands.registerCommand("PositionL2_L", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L2_L));
        // NamedCommands.registerCommand("PositionL2_R", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L2_R));
        // NamedCommands.registerCommand("PositionL3_L", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L3_L));
        // NamedCommands.registerCommand("PositionL3_R", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L3_R));
        // NamedCommands.registerCommand("PositionL4_L", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L4_L));
        // NamedCommands.registerCommand("PositionL4_R", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L4_R));

        NamedCommands.registerCommand("GoToPosition", AutomaticCommands.createGoToPositionCommand());

        NamedCommands.registerCommand("Score", createScoreCommand());
        NamedCommands.registerCommand("Pickup", createPickupCommand());
        // NamedCommands.registerCommand("Pickup", TESTcreatePickupCommand());

        // NamedCommands.registerCommand("GoToPositionAndScore", AutomaticCommands.createGoToPositionCommand().andThen(createScoreCommand()));
        NamedCommands.registerCommand("PrepareScore", createDynamicPrepareScoreCommand());

        try {
            var config = RobotConfig.fromGUISettings();
            m_driveSubsystem.configureAutoBuilder(config);
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }

        // FIXME: verify if we can register commands after configuring autobuilder (99% sure you can)
        NamedCommands.registerCommand("LocalizeToReefAB", createLocalizeToReefCommand(RelativeReefLocation.AB, true));
        NamedCommands.registerCommand("LocalizeToReefCD", createLocalizeToReefCommand(RelativeReefLocation.CD, true));
        NamedCommands.registerCommand("LocalizeToReefEF", createLocalizeToReefCommand(RelativeReefLocation.EF, true));
        NamedCommands.registerCommand("LocalizeToReefGH", createLocalizeToReefCommand(RelativeReefLocation.GH, true));
        NamedCommands.registerCommand("LocalizeToReefIJ", createLocalizeToReefCommand(RelativeReefLocation.IJ, true));
        NamedCommands.registerCommand("LocalizeToReefKL", createLocalizeToReefCommand(RelativeReefLocation.KL, true));

        NamedCommands.registerCommand("LocalizeToCoralStationLeft", createLocalizeToCoralStationCommand(CoralStationID.Left));
        NamedCommands.registerCommand("LocalizeToCoralStationRight", createLocalizeToCoralStationCommand(CoralStationID.Right));

        m_autoChooser = AutoBuilder.buildAutoChooser("thereisnoauto");
        m_autoChooser.addOption("thereisnoauto", Commands.none());
        m_autoChooser.addOption("Drive Wheel Radius Characterization", m_driveSubsystem
            .orientModules(CommandSwerveDrivetrain.getCircleOrientations())
            .andThen(new PrintCommand("modules oriented."))
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

    // private void setTargetReefLocation(RelativeReefLocation targetReefLocation) {
    //     m_targetReefLocation = targetReefLocation;
    //     SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
    // }

    private boolean m_robotCentricForward = false;
    private boolean m_robotCentricRight = false;
    private boolean m_robotCentricBackward = false;
    private boolean m_robotCentricLeft = false;
    private static final double robotCentricSpeed = 0.6;

    private void initialize() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // m_targetReefLocation = RelativeReefLocation.AB;
        RobotState.updateState(this);

        m_driveSubsystem.setDefaultCommand(
            m_driveSubsystem.applyRequest(() -> {
                    final double rotation = -DRIVER_CONTROLS.getRightX() * DriveConstants.MAX_ANGULAR_RATE;
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

        // setTargetReefLocation(m_targetReefLocation);
        // DRIVER_CONTROLS.incrementTargetReefLocation.onTrue(new InstantCommand(() -> {
        //     setTargetReefLocation(m_targetReefLocation.getNext());
        // }));
        // DRIVER_CONTROLS.decrementTargetReefLocation.onTrue(new InstantCommand(() -> {
        //     setTargetReefLocation(m_targetReefLocation.getPrevious());
        // }));

        // DRIVER_CONTROLS.localizeToReef.whileTrue(createLocalizeToReefCommand());

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
        OPERATOR_CONTROLS.INTAKE_IN.whileTrue(createPickupCommand());

        OPERATOR_CONTROLS.POSITION_CORAL_STATION.onTrue(AutomaticCommands.createInstantGoToPositionCommand(TargetScorePosition.CORAL_STATION));

        OPERATOR_CONTROLS.SCORE_L1.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L1));
        OPERATOR_CONTROLS.SCORE_L2_L.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L2_L));
        OPERATOR_CONTROLS.SCORE_L2_R.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L2_R));
        OPERATOR_CONTROLS.SCORE_L3_L.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L3_L));
        OPERATOR_CONTROLS.SCORE_L3_R.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L3_R));
        OPERATOR_CONTROLS.SCORE_L4_L.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L4_L));
        OPERATOR_CONTROLS.SCORE_L4_R.onTrue(AutomaticCommands.createGoToPositionCommand(TargetScorePosition.L4_R));

        OPERATOR_CONTROLS.SCORE_PIECE.whileTrue(createScoreCommand());

        OPERATOR_CONTROLS.ELEVATOR_MANUAL_UP.whileTrue(m_elevatorSubsystem.getManualUpCommand());
        OPERATOR_CONTROLS.ELEVATOR_MANUAL_DOWN.whileTrue(m_elevatorSubsystem.getManualDownCommand());

        OPERATOR_CONTROLS.GANTRY_MANUAL_LEFT.whileTrue(m_gantrySubsystem.getManualLeftCommand());
        OPERATOR_CONTROLS.GANTRY_MANUAL_RIGHT.whileTrue(m_gantrySubsystem.getManualRightCommand());

        OPERATOR_CONTROLS.CLIMBER_OUT.whileTrue(m_climbSubsystem.getManualActuatorOutCommand());
        OPERATOR_CONTROLS.CLIMBER_IN.whileTrue(m_climbSubsystem.getManualActuatorInCommand());

        OPERATOR_CONTROLS.CLIMBER_CLIMB.whileTrue(createSetClimbStateCommand(ClimbActuatorState.CLIMB));
        OPERATOR_CONTROLS.CLIMBER_HOME.whileTrue(createSetClimbStateCommand(ClimbActuatorState.HOME));

        OPERATOR_CONTROLS.CAGE_OUT.whileTrue(m_climbSubsystem.getCageOutCommand());
        OPERATOR_CONTROLS.CAGE_IN.whileTrue(m_climbSubsystem.getCageInCommand());

        OPERATOR_CONTROLS.CLIMB_OUT_AND_CAGE_OUT.whileTrue(createSetClimbStateCommand(ClimbActuatorState.CLIMB)
            .andThen(m_climbSubsystem.getAutomaticCageOutCommand())
        );

        OPERATOR_CONTROLS.ALGAE_HAND_OUT.whileTrue(m_algaeHandSubsystem.getManualOutCommand());
        // OPERATOR_CONTROLS.ALGAE_HAND_OUT.whileTrue(m_algaeHandSubsystem.getMiddleCommand());
        OPERATOR_CONTROLS.ALGAE_HAND_IN.whileTrue(m_algaeHandSubsystem.getManualInCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }

    public void update(Rotation2d rotation) {
        m_driveSubsystem.resetCustomEstimatedRotation(rotation);
        // SmartDashboard.putString("TARGET_TAG", OurUtils.formatReefLocation(m_targetReefLocation));
    }

    public void autonomousInit() {
        m_gantrySubsystem.resetManualPosition();
        m_gantrySubsystem.resetMotorPosition();

        if (RobotState.initialSwerveRotation != null)
            m_driveSubsystem.resetRotation(RobotState.initialSwerveRotation);
    }

    private TargetScorePosition m_targetScorePositionAfterAuto = null;

    public void autonomousExit() {
        m_targetScorePositionAfterAuto = RobotState.getTargetScorePosition();

        RobotState.setTargetScorePosition(TargetScorePosition.NONE);
        m_elevatorSubsystem.setIdle();
        m_gantrySubsystem.setIdle();
    }

    public void teleopInit() {
        if (m_targetScorePositionAfterAuto != null)
            RobotState.setTargetScorePosition(m_targetScorePositionAfterAuto);
        else
            RobotState.setTargetScorePosition(TargetScorePosition.NONE);

        m_targetScorePositionAfterAuto = null;

        // FIXME: better solutions for these
        m_elevatorSubsystem.resetManualPosition();
        m_gantrySubsystem.resetManualPosition();
    }

    public void robotPeriodic() {
        // SmartDashboard.putNumber("WHEEL_POSITION_0", m_driveSubsystem.getWheelRadiusCharacterizationPosition()[0]);
        // System.out.println("what?");

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
            double horizontalDifference = targetCentricTranslation.getY();
            horizontalDifference += RobotState.reefTargetHorizontalDistanceOffset;
            double forwardDifference = targetCentricTranslation.getX();
            SmartDashboard.putNumber("ReefTargetFowardDistanceInches", Units.metersToInches(forwardDifference));

            if (m_cameraSubsystem.getCanAutoAdjust())
                RobotState.setReefTargetHorizontalDistance(horizontalDifference);
            else
                RobotState.clearReefTargetHorizontalDistance();

            SmartDashboard.putNumber("ReefTargetHorizontalDistance", horizontalDifference);
        }
    }
}
