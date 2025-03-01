package com.spartronics4915.frc2025.commands.autos;

import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kPositionTolerance;
import static com.spartronics4915.frc2025.Constants.Drive.AutoConstants.kRotationTolerance;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.spartronics4915.frc2025.Constants.Drive;
import com.spartronics4915.frc2025.subsystems.SwerveSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class PositionPIDCommand extends Command{
    
    public SwerveSubsystem mSwerve;
    public final Pose2d goalPose;
    private PPHolonomicDriveController mDriveController = Drive.AutoConstants.kDriveController;

    private final Trigger endTrigger;

    private PositionPIDCommand(SwerveSubsystem mSwerve, Pose2d goalPose) {
        this.mSwerve = mSwerve;
        this.goalPose = goalPose;

        endTrigger = new Trigger(() -> {
            Pose2d diff = mSwerve.getPose().relativeTo(goalPose);
            return MathUtil.isNear(
                0.0, 
                diff.getRotation().getRotations(), 
                kRotationTolerance.getRotations(), 
                0.0, 
                1.0
            ) && diff.getTranslation().getNorm() < kPositionTolerance.in(Meters);
        }).debounce(0.1);
    }

    public static Command generateCommand(SwerveSubsystem swerve, Pose2d goalPose, Time timeout){
        return new PositionPIDCommand(swerve, goalPose).withTimeout(timeout);
    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        mSwerve.drive(
            mDriveController.calculateRobotRelativeSpeeds(
                mSwerve.getPose(), goalState
            )
        );
    }

    @Override
    public boolean isFinished() {
        return endTrigger.getAsBoolean();
    }
}
