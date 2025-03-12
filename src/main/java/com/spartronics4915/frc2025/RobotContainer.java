// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.spartronics4915.frc2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.spartronics4915.frc2025.Constants.ArmConstants.ArmSubsystemState;
import com.spartronics4915.frc2025.Constants.ElevatorConstants.ElevatorSubsystemState;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;
import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.Constants.OI;
import com.spartronics4915.frc2025.subsystems.ClimberSubsystem;
import com.spartronics4915.frc2025.subsystems.MechanismRenderer;
import com.spartronics4915.frc2025.subsystems.MotorSimulationSubsystem;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.subsystems.WinchClimber;
import com.spartronics4915.frc2025.subsystems.coral.IntakeSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ArmSubsystem;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;
import com.spartronics4915.frc2025.util.ModeSwitchHandler;
import com.spartronics4915.frc2025.subsystems.coral.ElevatorSubsystem;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(Drive.SwerveDirectories.COMP_CHASSIS);

    private static final CommandXboxController driverController = new CommandXboxController(OI.kDriverControllerPort);

    private static final CommandXboxController operatorController = new CommandXboxController(
        OI.kOperatorControllerPort);
        
    private static final CommandXboxController debugController = new CommandXboxController(OI.kDebugControllerPort);

    private static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    
    public final IntakeSubsystem intakeSubsystem;
    public final ArmSubsystem armSubsystem;
    public final ElevatorSubsystem elevatorSubsystem;
    public final WinchClimber climberSubsystem;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        intakeSubsystem = new IntakeSubsystem();
        armSubsystem = new ArmSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        climberSubsystem = new WinchClimber();

        ModeSwitchHandler.EnableModeSwitchHandler(
            intakeSubsystem,
            armSubsystem,
            elevatorSubsystem
        ); //TODO add any subsystems that extend ModeSwitchInterface

        MechanismRenderer.generateRenderer(
            elevatorSubsystem::getDesiredPosition, 
            () -> armSubsystem.getTargetPosition().getMeasure(), 
            intakeSubsystem::getSpeed, 
            intakeSubsystem::detect,
            "Target Position"
        );

        MechanismRenderer.generateRenderer(
            () -> Meters.of(elevatorSubsystem.getPosition()), 
            () -> armSubsystem.getPosition().getMeasure(), 
            intakeSubsystem::getSpeed, 
            intakeSubsystem::detect,
            "Current Position"
        );

        MechanismRenderer.generateRenderer(
            () -> elevatorSubsystem.getSetpoint(), 
            () -> armSubsystem.getSetpoint().getMeasure(), 
            intakeSubsystem::getSpeed, 
            intakeSubsystem::detect,
            "setpoints"
        );

        // Configure the trigger bindings
        configureBindings();

       

        // Need to initialize this here after vision is configured.
        // Need to clean up initialization flow to make it more clear
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {

        //#endregion

        //#region Operator Controls

        operatorController.back().onTrue(
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.FUNNEL_UNSTUCK)
        ).onFalse(
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.IN)
        ); //windows button


        operatorController.povUp().whileTrue(elevatorSubsystem.manualMode(0.002));

        operatorController.povDown().whileTrue(elevatorSubsystem.manualMode(-0.002));

        operatorController.povLeft().whileTrue(armSubsystem.manualMode(Rotation2d.fromDegrees(-0.3)));

        operatorController.povRight().whileTrue(armSubsystem.manualMode(Rotation2d.fromDegrees(0.3)));
        
        operatorController.rightBumper()
            .whileTrue(climberSubsystem.driveWinch(0.5).withName("Move Climber Pos"));
            // .onTrue(dynamics.gotoClimb());

        operatorController.leftBumper()
            .whileTrue(climberSubsystem.driveWinch(-0.5).withName("Move Climber Neg"));
            // .onTrue(dynamics.gotoClimb());

        //#endregion

        SmartDashboard.putData("setPreset1", armSubsystem.setMechanismAngleCommand(Rotation2d.fromDegrees(270)));

        SmartDashboard.putNumber("elevator setpoint", 0);
        SmartDashboard.putNumber("arm setpoint", 270);

        SmartDashboard.putData("ManualElevator", Commands.defer(() -> {
            return elevatorSubsystem.setSetPointCommand(SmartDashboard.getNumber("elevator setpoint", 0.0));
        }, Set.of()));
        SmartDashboard.putData("ManualArm", Commands.defer(() -> {
            return armSubsystem.setSetpointCommand(Rotation2d.fromDegrees(SmartDashboard.getNumber("arm setpoint", 270.0)));
        }, Set.of()));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    private SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<Command>();

        NamedCommands.registerCommand("print", Commands.print("ping"));

        chooser.setDefaultOption("None", Commands.none());


        chooser.onChange((c) -> {
            SmartDashboard.putBoolean("Using Variable Auto?", c.getName() == "variableAuto");
        });


        SmartDashboard.putData("Auto Chooser", chooser);

        return chooser;
    }

    public static CommandXboxController getDriveController() {
        return driverController;
    }

    public static CommandXboxController getOperatorController() {
        return operatorController;
    }

    public static CommandXboxController getDebugController() {
        return debugController;
    }

    public static AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

}
