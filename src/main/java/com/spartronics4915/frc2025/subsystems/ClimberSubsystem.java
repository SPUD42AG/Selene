package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.spartronics4915.frc2025.Constants;
import com.spartronics4915.frc2025.Constants.ClimberConstants;
import com.spartronics4915.frc2025.Constants.ClimberConstants.ClimberState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {


    private SparkMax mClimberMotor;
    private SparkMaxConfig config;
    private RelativeEncoder mClimberEncoder;
    private TrapezoidProfile mClimberProfile;


    private ArmFeedforward mFFCalculator;
    private Rotation2d mCurrentSetPoint = Rotation2d.fromRotations(0);
    private State mCurrentState;
    
    private SparkClosedLoopController mClosedLoopController;
    
    public ClimberSubsystem () {                
        
        mClimberMotor = new SparkMax(Constants.ClimberConstants.motorID, MotorType.kBrushless);
            config = new SparkMaxConfig();

                config
                    .inverted(true)
                    .idleMode(IdleMode.kBrake);
                config.encoder
                    .positionConversionFactor(1000)
                    .velocityConversionFactor(1000);
                config.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(Constants.ClimberConstants.kP, Constants.ClimberConstants.kI, Constants.ClimberConstants.kD);
    
                mClimberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                mClimberEncoder = mClimberMotor.getEncoder();
        
        mFFCalculator = new ArmFeedforward(ClimberConstants.kS,ClimberConstants.kG,ClimberConstants.kV,ClimberConstants.kS);

        initClimberProfile();
        
        initClosedLoopController();

        resetMechanism();
    }

    private Rotation2d convertRaw(double rotation) {
        Rotation2d angle = Rotation2d.fromDegrees(rotation);
        return angle;
    }

    private double angleToRaw(Rotation2d angle) {
        double rotation = angle.getRotations();
        return rotation;
    }

    public void resetMechanism(){
        Rotation2d position = getPosition();
        mCurrentSetPoint = position;
        mCurrentState = new State(angleToRaw(position), 0.0);
    }

    private void setMechanismAngle(Rotation2d angle){
        mClimberEncoder.setPosition(angleToRaw(angle));
        resetMechanism();
    }

    @Override
    public void periodic() {
        
        //need set points as a input
        mCurrentSetPoint = Rotation2d.fromRotations(
            MathUtil.clamp(mCurrentSetPoint.getRotations(), ClimberConstants.kMinAngle.getRotations(), ClimberConstants.kMaxAngle.getRotations()));

        mCurrentState = mClimberProfile.calculate(ClimberConstants.kDt, mCurrentState, new State((angleToRaw(mCurrentSetPoint)), 0.0));

        mClosedLoopController.setReference(mCurrentState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, mFFCalculator.calculate(mCurrentState.position, mCurrentState.velocity));
        
        


    }

    public void setSetpoint(Rotation2d newSetpoint){
        mCurrentSetPoint = newSetpoint;
    }

    public void setSetpoint(ClimberState newState) {
        setSetpoint(newState.angle);
    }

    public void incrementAngle(Rotation2d delta){
        mCurrentSetPoint = mCurrentSetPoint.plus(delta);
    }

    private Rotation2d getPosition() {
        double position = mClimberMotor.getEncoder().getPosition();

        return convertRaw(position);
    }

    private void initClosedLoopController() {
        mClosedLoopController = mClimberMotor.getClosedLoopController();
    }

     private void initClimberProfile() {
        mClimberProfile = new TrapezoidProfile(ClimberConstants.kConstraints);
        mCurrentState = new State(angleToRaw(getPosition()), 0.0);
    }

    private double getVelocity() {
        double velocity = mClimberMotor.getEncoder().getVelocity();
        return velocity;
    }


    public Command manualMode(Rotation2d delta){
        return this.runEnd(() -> {
            incrementAngle(delta);
        }, () -> {
            resetMechanism();
        });
    }

    public Command setSetpointCommand(Rotation2d newSetpoint){
        return this.runOnce(() -> setSetpoint(newSetpoint));
    }

    public Command presetCommand(ClimberState preset){
        return setSetpointCommand(preset.angle);
    }

    public Command setMechanismAngleCommand(Rotation2d newAngle){
        return this.runOnce(() -> setMechanismAngle(newAngle));
    }
      
    public void onModeSwitch() {
        resetMechanism();
    }
    
}