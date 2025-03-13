package com.spartronics4915.frc2025.subsystems.vision;

import com.spartronics4915.frc2025.LimelightHelpers;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightModel;
import com.spartronics4915.frc2025.Constants.VisionConstants.LimelightRole;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;
import com.spartronics4915.frc2025.util.Structures.LimelightConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightDevice extends SubsystemBase{
    
    private final String name;
    private final LimelightModel model;
    private final int id;
    private final LimelightRole role;
    private int[] tagFilter = new int[]{};

    double robotYaw = getYaw();
    
    public LimelightDevice(LimelightConstants constants) {
            this.name = "limelight-" + constants.name();
            this.model = constants.model();
            this.id = constants.id();
            this.role = constants.role();

    }   
    
    public double getTx() {
        return LimelightHelpers.getTX(name);
    }

    public double getTy() {
        return LimelightHelpers.getTY(name);
    }

    public boolean getTv() {
        return LimelightHelpers.getTV(name);
    }

    public void SetRobotOrientation() {
        LimelightHelpers.SetRobotOrientation(name, robotYaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    @Override
        public void periodic() {
        
                LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
                    if (limelightMeasurement.tagCount >= 2) {  // Only trust measurement if we see multiple tags
                        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
                        m_poseEstimator.addVisionMeasurement(
                        limelightMeasurement.pose,
                        limelightMeasurement.timestampSeconds
                    );


        super.periodic();
    }
}
    




