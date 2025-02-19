package com.spartronics4915.frc2025.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static com.spartronics4915.frc2025.Constants.WinchClimberConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WinchClimber extends SubsystemBase {
    
    private final SparkBase mMotor;
    private final RelativeEncoder mEncoder;

    public WinchClimber() {
        super();

        mMotor = new SparkMax(kMotorID, MotorType.kBrushless);
        mMotor.configure(kMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        mEncoder = mMotor.getEncoder();
    }

}
