// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
import frc.robot.subsystems.SubsystemInterfaces.ElevatorSubsystemInterface.ElevatorState;
import frc.robot.subsystems.SubsystemInterfaces.GantrySubsystemInterface.GantryState;

public class RobotContainer {
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric m_fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MAX_SPEED * 0.1).withRotationalDeadband(DriveConstants.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake m_brakeRequest = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric m_robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final boolean shouldRunWheelRadiusCharacterization = true;

    private final CommandSwerveDrivetrain m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsystem;

    private final ElevatorSubsystemInterface m_elevatorSubsystem;
    private final GantrySubsystemInterface m_gantrySubsystem;
    private final IntakeOuttakeSubsystemInterface m_intakeOuttakeSubsystem;
    private final ClimbSubsystemInterface m_climbSubsystem;
    private final AlgaeHandSubsystemInterface m_algaeHandSubsystem;

    private Command createLocalizeToReefCommand() {
        return new CameraSubsystem.DynamicCommand(() -> {
            return m_cameraSubsystem.getPathCommandFromReefTag(m_targetReefLocation);
        });
    }
    private Command createScoreCommand() {
        return m_intakeOuttakeSubsystem.addToCommandRequirements(new Command() {
            @Override
            public void initialize() {
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
    private static class PickupCommand extends Command {
        @Override
        public void initialize() {
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
    // private static final class AutoPickupCommand extends PickupCommand {
    //     private GantrySubsystemInterface m_gantrySubsystem;

    //     public AutoPickupCommand(GantrySubsystemInterface gantrySubsystem) {
    //         m_gantrySubsystem = gantrySubsystem;
    //     }

    //     @Override
    //     public boolean isFinished() {
    //         return m_gantrySubsystem.getScoreEnterSensorTripped();
    //     }

    //     @Override
    //     public void end(boolean isInterrupted) { 
    //         // do nothing: we don't want the intake to stop here; it will be stopped from auto
    //     }
    // }
    private Command createPickupCommand() {
        return m_intakeOuttakeSubsystem.addToCommandRequirements(
            new PickupCommand()
        );
    }

    // private Command TESTcreatePickupCommand() {
    //     return m_intakeOuttakeSubsystem.addToCommandRequirements(
    //         new AutoPickupCommand(m_gantrySubsystem)
    //     );
    // }

    private Command createSetClimbStateCommand(ClimbActuatorState climbState) {
        return m_climbSubsystem.addToCommandRequirements(new InstantCommand(() -> {
            RobotState.setClimbActuatorState(climbState);
        }));
    }

    private final SendableChooser<Command> m_autoChooser;

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
        NamedCommands.registerCommand("GoToCoralStation", AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.CORAL_STATION));

        NamedCommands.registerCommand("PositionL1", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L1));
        NamedCommands.registerCommand("PositionL2_L", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L2_L));
        NamedCommands.registerCommand("PositionL2_R", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L2_R));
        NamedCommands.registerCommand("PositionL3_L", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L3_L));
        NamedCommands.registerCommand("PositionL3_R", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L3_R));
        NamedCommands.registerCommand("PositionL4_L", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L4_L));
        NamedCommands.registerCommand("PositionL4_R", AutomaticCommands.createSetTargetScorePositionCommand(TargetScorePosition.L4_R));

        NamedCommands.registerCommand("GoToPosition", AutomaticCommands.createGoToPositionCommand());
        // NamedCommands.registerCommand("GoToPosition", AutomaticCommands.TESTcreateGoToPositionCommand());

        NamedCommands.registerCommand("Score", createScoreCommand());
        NamedCommands.registerCommand("Pickup", createPickupCommand());
        // NamedCommands.registerCommand("Pickup", TESTcreatePickupCommand());

        NamedCommands.registerCommand("LocalizeToReefAB", AutomaticCommands.createLocalizeToReefCommand(RelativeReefLocation.AB));
        NamedCommands.registerCommand("LocalizeToReefCD", AutomaticCommands.createLocalizeToReefCommand(RelativeReefLocation.CD));
        NamedCommands.registerCommand("LocalizeToReefEF", AutomaticCommands.createLocalizeToReefCommand(RelativeReefLocation.EF));
        NamedCommands.registerCommand("LocalizeToReefGH", AutomaticCommands.createLocalizeToReefCommand(RelativeReefLocation.GH));
        NamedCommands.registerCommand("LocalizeToReefIJ", AutomaticCommands.createLocalizeToReefCommand(RelativeReefLocation.IJ));
        NamedCommands.registerCommand("LocalizeToReefKL", AutomaticCommands.createLocalizeToReefCommand(RelativeReefLocation.KL));

        m_driveSubsystem.configureAutoBuilder();

        m_autoChooser = AutoBuilder.buildAutoChooser("thereisnoauto");
        m_autoChooser.addOption("thereisnoauto", Commands.none());
        m_autoChooser.addOption("Drive Wheel Radius Characterization", m_driveSubsystem
            .orientModules(CommandSwerveDrivetrain.getCircleOrientations())
            .andThen(new PrintCommand("modules oriented."))
            .andThen(new WheelRadiusCharacterization(m_driveSubsystem, Direction.COUNTER_CLOCKWISE)));
        SmartDashboard.putData("AutoChooser", m_autoChooser);

        initialize();
    }

    private void setTargetReefLocation(RelativeReefLocation targetReefLocation) {
        m_targetReefLocation = targetReefLocation;
        SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
    }

    private boolean m_robotCentricForward = false;
    private boolean m_robotCentricRight = false;
    private boolean m_robotCentricBackward = false;
    private boolean m_robotCentricLeft = false;
    private static final double robotCentricSpeed = 0.6;

    private RelativeReefLocation m_targetReefLocation;
    private void initialize() {
        DriverStation.silenceJoystickConnectionWarning(true);
        m_targetReefLocation = RelativeReefLocation.AB;
        RobotState.updateState(this);

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
                            .withRotationalRate(-DRIVER_CONTROLS.getRightX() * DriveConstants.MAX_ANGULAR_RATE);
                    } else
                        return m_fieldCentricRequest.withVelocityX(-DRIVER_CONTROLS.getLeftY() * DriveConstants.MAX_SPEED)
                            .withVelocityY(-DRIVER_CONTROLS.getLeftX() * DriveConstants.MAX_SPEED)
                            .withRotationalRate(-DRIVER_CONTROLS.getRightX() * DriveConstants.MAX_ANGULAR_RATE);
                }
            )
        );

        // DRIVER CONTROLS

        DRIVER_CONTROLS.brake.whileTrue(m_driveSubsystem.applyRequest(() -> m_brakeRequest));
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

        setTargetReefLocation(m_targetReefLocation);
        DRIVER_CONTROLS.incrementTargetReefLocation.onTrue(new InstantCommand(() -> {
            setTargetReefLocation(m_targetReefLocation.getNext());
        }));
        DRIVER_CONTROLS.decrementTargetReefLocation.onTrue(new InstantCommand(() -> {
            setTargetReefLocation(m_targetReefLocation.getPrevious());
        }));

        DRIVER_CONTROLS.localizeToReef.whileTrue(createLocalizeToReefCommand());

        // OPERATOR CONTROLS

        OPERATOR_CONTROLS.INTAKE_OUT.whileTrue(m_intakeOuttakeSubsystem.addToCommandRequirements(Commands.runEnd(() -> {
            RobotState.startIntake(IntakeState.OUT);
        }, () -> {
            RobotState.stopIntake();
        })));
        OPERATOR_CONTROLS.INTAKE_IN.whileTrue(createPickupCommand());

        OPERATOR_CONTROLS.POSITION_CORAL_STATION.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.CORAL_STATION));

        OPERATOR_CONTROLS.SCORE_L1.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.L1));
        OPERATOR_CONTROLS.SCORE_L2_L.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.L2_L));
        OPERATOR_CONTROLS.SCORE_L2_R.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.L2_R));
        OPERATOR_CONTROLS.SCORE_L3_L.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.L3_L));
        OPERATOR_CONTROLS.SCORE_L3_R.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.L3_R));
        OPERATOR_CONTROLS.SCORE_L4_L.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.L4_L));
        OPERATOR_CONTROLS.SCORE_L4_R.onTrue(AutomaticCommands.createSetAndGoToTargetScorePositionCommand(TargetScorePosition.L4_R));

        OPERATOR_CONTROLS.SCORE_PIECE.whileTrue(createScoreCommand());

        OPERATOR_CONTROLS.ELEVATOR_MANUAL_UP.whileTrue(m_elevatorSubsystem.getManualUpCommand());
        OPERATOR_CONTROLS.ELEVATOR_MANUAL_DOWN.whileTrue(m_elevatorSubsystem.getManualDownCommand());

        OPERATOR_CONTROLS.GANTRY_MANUAL_LEFT.whileTrue(m_gantrySubsystem.getManualLeftCommand());
        OPERATOR_CONTROLS.GANTRY_MANUAL_RIGHT.whileTrue(m_gantrySubsystem.getManualRightCommand());

        // FIXME: why doesn't new InstantCommand(gantry::resetPositionStuff) work??
        OPERATOR_CONTROLS.GANTRY_RESET_POSITION.onTrue(new InstantCommand(() -> {
            m_gantrySubsystem.resetManualPosition();
            m_gantrySubsystem.resetMotorPosition();
        }));

        OPERATOR_CONTROLS.CLIMBER_OUT.whileTrue(m_climbSubsystem.getManualActuatorOutCommand());
        OPERATOR_CONTROLS.CLIMBER_IN.whileTrue(m_climbSubsystem.getManualActuatorInCommand());

        OPERATOR_CONTROLS.CLIMBER_CLIMB.whileTrue(createSetClimbStateCommand(ClimbActuatorState.CLIMB));
        OPERATOR_CONTROLS.CLIMBER_HOME.whileTrue(createSetClimbStateCommand(ClimbActuatorState.HOME));

        OPERATOR_CONTROLS.CAGE_OUT.whileTrue(m_climbSubsystem.getCageOutCommand());
        OPERATOR_CONTROLS.CAGE_IN.whileTrue(m_climbSubsystem.getCageInCommand());

        OPERATOR_CONTROLS.ALGAE_HAND_OUT.whileTrue(m_algaeHandSubsystem.getManualOutCommand());
        // OPERATOR_CONTROLS.ALGAE_HAND_OUT.whileTrue(m_algaeHandSubsystem.getMiddleCommand());
        OPERATOR_CONTROLS.ALGAE_HAND_IN.whileTrue(m_algaeHandSubsystem.getManualInCommand());
    }

    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
        // return m_elevatorSubsystem.getTempGoUntilTargetIncreaseCommand(2.54).andThen(m_elevatorSubsystem.getTempHoldPositionCommand());
        // return m_elevatorSubsystem.getTimeTravelCommand(4.2);
    }

    public void update(Rotation2d rotation) {
        m_driveSubsystem.resetCustomEstimatedRotation(rotation);
        SmartDashboard.putNumber("TARGET_TAGID", m_targetReefLocation.getTagID());
    }

    public void autonomousInit() {
        m_gantrySubsystem.resetManualPosition();
        m_gantrySubsystem.resetMotorPosition();

        if (RobotState.initialSwerveRotation != null)
            m_driveSubsystem.resetRotation(RobotState.initialSwerveRotation);
    }

    public void autonomousExit() {
        RobotState.setTargetScorePosition(TargetScorePosition.NONE);
        m_elevatorSubsystem.setIdle();
        m_gantrySubsystem.setIdle();
    }

    final boolean testingthiswervethingplsdelete = false;
    final TalonFX frontleftdrive = new TalonFX(20, Constants.CANIVORE_NAME);
    final TalonFX frontrightdrive = new TalonFX(21, Constants.CANIVORE_NAME);
    final TalonFX backleftdrive = new TalonFX(23, Constants.CANIVORE_NAME);
    final TalonFX backrightdrive = new TalonFX(22, Constants.CANIVORE_NAME);
    double frontleftrotations = 0;
    double frontrightrotations = 0;
    double backleftrotations = 0;
    double backrightrotations = 0;
    public void teleopInit() {
        RobotState.setTargetScorePosition(TargetScorePosition.NONE);

        m_elevatorSubsystem.resetManualPosition();
        if (testingthiswervethingplsdelete) {
            frontleftrotations = frontleftdrive.getPosition().getValueAsDouble();
            frontrightrotations = frontrightdrive.getPosition().getValueAsDouble();
            backleftrotations = backleftdrive.getPosition().getValueAsDouble();
            backrightrotations = backrightdrive.getPosition().getValueAsDouble();
            SmartDashboard.putNumber("FRONTLEFT_DISTANCE", 0);
            SmartDashboard.putNumber("FRONTRIGHT_DISTANCE", 0);
            SmartDashboard.putNumber("BACKLEFT_DISTANCE", 0);
            SmartDashboard.putNumber("BACKRIGHT_DISTANCE", 0);
        }
    }

    public void robotPeriodic() {
        if (testingthiswervethingplsdelete) {
            // SmartDashboard.putNumber("SWERVEDISTANCE", m_driveSubsystem.getCustomEstimatedPose().getY());
            SmartDashboard.putNumber("FRONTLEFT_DISTANCE", frontleftdrive.getPosition().getValueAsDouble() - frontleftrotations);
            SmartDashboard.putNumber("FRONTRIGHT_DISTANCE", frontrightdrive.getPosition().getValueAsDouble() - frontrightrotations);
            SmartDashboard.putNumber("BACKLEFT_DISTANCE", backleftdrive.getPosition().getValueAsDouble() - backleftrotations);
            SmartDashboard.putNumber("BACKRIGHT_DISTANCE", backrightdrive.getPosition().getValueAsDouble() - backrightrotations);
        }

        final Pose2d robotPose = m_driveSubsystem.getCustomEstimatedPose();
        Pose2d targetPose = m_targetReefLocation.getPose();
        if (targetPose != null) {
            double closest_distance = 1000;
            for (var location : RelativeReefLocation.values()) {
                var pose = location.getPose();
                if (pose == null)
                    continue;
                final double distance = pose.getTranslation().getDistance(robotPose.getTranslation());
                if (distance < closest_distance) {
                    closest_distance = distance;
                    targetPose = pose;
                }
            }

            final Translation2d targetCentricTranslation = robotPose.getTranslation().minus(targetPose.getTranslation())
                .rotateBy(targetPose.getRotation().unaryMinus());
            final double horizontalDifference = targetCentricTranslation.getY();

            if (m_cameraSubsystem.getCanAutoAdjust())
                RobotState.setReefTargetHorizontalDistance(horizontalDifference);
            else
                RobotState.clearReefTargetHorizontalDistance();

            SmartDashboard.putNumber("ReefTargetHorizontalDistance", horizontalDifference);
        }
    }
}
