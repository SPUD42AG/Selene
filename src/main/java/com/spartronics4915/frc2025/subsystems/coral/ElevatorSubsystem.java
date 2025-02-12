package com.spartronics4915.frc2025.subsystems.coral;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.Constants.ElevatorConstants;
import com.spartronics4915.frc2025.Constants.ElevatorConstants.ElevatorSubsystemState;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ElevatorSubsystem extends SubsystemBase implements ModeSwitchInterface{

    private SparkMax motor;
    private SparkMaxConfig motorConfig;
    private SparkMax follower;
    private RelativeEncoder motorEncoder;
    private SparkClosedLoopController elevatorClosedLoopController;
    private ElevatorFeedforward FFCalculator;

    private TrapezoidProfile elevatorProfile;

    private double currentSetPoint;
    private State currentState;
    
    public ElevatorSubsystem() {
        // Main elevator motor init
        motor = new SparkMax(ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

        motorEncoder = motor.getEncoder();

        motorConfig = new SparkMaxConfig();

        motorConfig
            // .inverted(ElevatorConstants.motorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(ElevatorConstants.motorSmartCurrentLimit)
            .secondaryCurrentLimit(ElevatorConstants.motorSecondaryCurrentLimit);
        motorConfig.encoder
            .positionConversionFactor(ElevatorConstants.motorPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.motorVelocityConversionFactor);
        motorConfig
            .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.motorPIDConstants.kP, ElevatorConstants.motorPIDConstants.kI, ElevatorConstants.motorPIDConstants.kD);
            
        

        // Follower motor init
        follower = new SparkMax(ElevatorConstants.elevatorFollowerID, MotorType.kBrushless);

        SparkMaxConfig followerConfig = new SparkMaxConfig();

        // followerConfig
            // .inverted(ElevatorConstants.followerInverted)
        //     .idleMode(IdleMode.kBrake)
        //     .smartCurrentLimit(ElevatorConstants.followerSmartCurrentLimit)
        //     .secondaryCurrentLimit(ElevatorConstants.followerSecondaryCurrentLimit);
        // followerConfig.encoder
        //     .positionConversionFactor(ElevatorConstants.followerPositionConversionFactor)
        //     .velocityConversionFactor(ElevatorConstants.followerVelocityConversionFactor);
        // followerConfig
        //     .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //     .pid(ElevatorConstants.followerPIDConstants.kP, ElevatorConstants.followerPIDConstants.kI, ElevatorConstants.followerPIDConstants.kD);

        followerConfig.follow(ElevatorConstants.elevatorMotorID, true);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        follower.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        FFCalculator = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
        elevatorProfile = new TrapezoidProfile(ElevatorConstants.constraints);
        elevatorClosedLoopController = motor.getClosedLoopController();

        setMechanismPosition(0.0);
    }

    public void resetMechanism() {
        resetMechanism(getPosition());
    }

    public void resetMechanism(double position) {
        currentSetPoint = position;
        currentState = new State(position, 0);
    }

    private double getPosition() {
        return motorEncoder.getPosition();
    }

    private double getVelocity() {
        return motorEncoder.getVelocity();
    }

    private final DoublePublisher setpointPub = NetworkTableInstance.getDefault().getTable("loggingElev").getDoubleTopic("setpoint").publish();
    private final DoublePublisher appliedOutPub = NetworkTableInstance.getDefault().getTable("loggingElev").getDoubleTopic("appliedOut").publish();
    private final DoublePublisher appliedOutFollowPub = NetworkTableInstance.getDefault().getTable("loggingElev").getDoubleTopic("appliedOutFollow").publish();

    private final DoublePublisher currStatePub = NetworkTableInstance.getDefault().getTable("loggingElev").getDoubleTopic("currState").publish();
    private final DoublePublisher PositionPub = NetworkTableInstance.getDefault().getTable("loggingElev").getDoubleTopic("Position").publish();


    @Override
    public void periodic() {

        currentSetPoint = MathUtil.clamp(
            currentSetPoint, 
            ElevatorConstants.minHeight, 
            ElevatorConstants.maxHeight
        );

        currentState = elevatorProfile.calculate(ElevatorConstants.dt, currentState, new State(currentSetPoint, 0));

        elevatorClosedLoopController.setReference(currentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, FFCalculator.calculate(currentState.velocity));

        setpointPub.accept(currentSetPoint);
        appliedOutPub.accept(motor.getAppliedOutput());
        currStatePub.accept(currentState.position);
        PositionPub.accept(getPosition());
        appliedOutFollowPub.accept(follower.getAppliedOutput());
    }

    public void moveToPosition(ElevatorSubsystemState value) {
        currentSetPoint = value.meter;
    }

    public void setSetPoint(double setPoint) {
        currentSetPoint = setPoint;
    }

    private void setMechanismPosition(double position){
        motorEncoder.setPosition(position);
        resetMechanism(position);
    }

    public void incrementAngle(double delta){
        currentSetPoint += delta;
    }

    //commands

    public Command manualMode(double delta){
        return this.runEnd(() -> {
            incrementAngle(delta);
        }, () -> {
            resetMechanism();
        });
    }

    public Command setSetPointCommand(double newSetPoint){
        return this.runOnce(() -> setSetPoint(newSetPoint));
    }

    public Command presetCommand(ElevatorSubsystemState preset){
        return this.runOnce(() -> moveToPosition(preset));
    }

    public Command setMechanismAngleCommand(double position){
        return this.runOnce(() -> setMechanismPosition(position));
    }

    @Override
    public void onModeSwitch() {
        resetMechanism();
    }
}
