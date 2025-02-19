package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.spartronics4915.frc2025.util.ModeSwitchHandler.ModeSwitchInterface;

import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchClimber extends SubsystemBase implements ModeSwitchInterface{
    
    private final SparkBase mMotor;
    private final RelativeEncoder mEncoder;

    private double mSpeedSetpoint = 0.0;
    private boolean mOverrideRetractedLimit = false;

    public WinchClimber() {
        super();

        mMotor = new SparkMax(kMotorID, MotorType.kBrushless);
        mMotor.configure(kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mEncoder = mMotor.getEncoder();

        mEncoder.setPosition(kStartingAngle.getRotations());

        mMotor.set(0.0);
    }

    public void setWinchSpeed(double speed){
        mSpeedSetpoint = speed;
    }

    public void setOverride(boolean override){
        mOverrideRetractedLimit = override;
    }

    public void stopWinch(){
        mMotor.set(0.0);
        mSpeedSetpoint = 0.0;
    }

    @Override
    public void periodic() {
        softLimitUpdate();
        
        mMotor.set(mSpeedSetpoint);
    }

    private void softLimitUpdate(){
        Rotation2d pos = Rotation2d.fromRotations(mEncoder.getPosition());
        boolean movingForward = mSpeedSetpoint > 0.0; //moving to engage the cage

        if (movingForward && pos.getDegrees() > kEngagedAngle.getDegrees()) {
            mSpeedSetpoint = 0.0;
        }
        
        if (!movingForward && pos.getDegrees() < kRetractedAngle.getDegrees() && !mOverrideRetractedLimit) { 
            mSpeedSetpoint = 0.0;
        }

        if (!movingForward && pos.getDegrees() < kStartingAngle.getDegrees() && !mOverrideRetractedLimit) { //prevents too far in the wrong direction
            mSpeedSetpoint = 0.0;
        }
    }

    @Override public void onModeSwitch() {stopWinch();}
    @Override public void onDisable() {stopWinch();}

}
