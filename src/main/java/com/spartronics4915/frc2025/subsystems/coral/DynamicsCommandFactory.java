package com.spartronics4915.frc2025.subsystems.coral;
import com.pathplanner.lib.auto.NamedCommands;
import com.spartronics4915.frc2025.RobotContainer;
import com.spartronics4915.frc2025.Constants.IntakeConstants.IntakeSpeed;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static com.spartronics4915.frc2025.Constants.DynamicsConstants.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.Radians;

public class DynamicsCommandFactory {

    private IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    private LaserCan funnelLC;

    public DynamicsCommandFactory(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.funnelLC = new LaserCan(kFunnelLaserCanID);

        if (RobotBase.isReal()) {
            throw new RuntimeException("This is currently only working in sim, change the way it gets the position");
        }
    }

    private record DynamicsSetpoint(double heightMeters, Rotation2d armAngle) {
    }

    public enum DynaPreset{
        LOAD(0.0, Rotation2d.fromDegrees(122.210182)),
        PRESCORE(0.0, Rotation2d.fromDegrees(245.826889)),
        // L2(0.0, Rotation2d.fromDegrees(319.357058)),
        L3(Inches.of(21.826948).in(Meters), Rotation2d.fromDegrees(301.896888)),
        L4(Inches.of(53.312356).in(Meters), Rotation2d.fromDegrees(359));

        private final DynamicsSetpoint setpoint;

        private DynaPreset(double meters, Rotation2d angle) {
            this.setpoint = new DynamicsSetpoint(meters, angle);
        }
    }

    //#region Composite Commands


    //TODO swap out target with actual position

    /**
     * 
     * @return whether the elevator is safe to move (based on the arm's position)
     */
    private boolean isElevSafeToMove(){
        var currAngle =  armSubsystem.getTargetPosition();
        return currAngle.getCos() < Math.cos(kMoveableArmAngle.in(Radians));
    }

    private boolean isElevAtSetpoint(double setpoint){
        System.out.println(Math.abs(setpoint - elevatorSubsystem.getDesiredPosition().in(Meters)));
        return Math.abs(setpoint - elevatorSubsystem.getDesiredPosition().in(Meters)) < 2*kElevatorHeightTolerance;
    }

    private boolean isArmAtSetpoint(Rotation2d angle){
        return armSubsystem.getTargetPosition().minus(angle).getMeasure().isNear(Degrees.of(0), kArmAngleTolerance);
    }

    // is the arm below the horizon and is the elevator too low to allow movement
    private boolean isArmStowed(){
        return (armSubsystem.getTargetPosition().getDegrees() < 180) && isElevStowed();
    }

    private boolean isElevStowed(){
        return  elevatorSubsystem.getDesiredPosition().in(Meters) + kElevatorHeightTolerance < kMinSafeElevHeight;
    }

    private boolean isCoralInArm(){
        // return false;
        return intakeSubsystem.detect();
    }

    public boolean funnelDetect(){

        var measurement = funnelLC.getMeasurement();

        if (measurement == null) {
            return false;
        }

        return  measurement.distance_mm < funnelLCTriggerDist.in(Millimeter) || intakeSubsystem.detect(); // the || is here as a way to prevent us stalling at a CS when we are already holding a coral
    }

    /**
     * Make sure the elevator is not in the load position, if it is goto the safe Elevator height, otherwise it's fine to move the arm
     * Then move the arm to make it safe to move
     * 
     * @return Command to do above actions
     */
    private Command makeSystemSafeToMove(boolean forceMinSafeHeightMove){ 
        //note to self, careful about when data gets read here
        return Commands.either(
            Commands.sequence(
                Commands.either(
                    elevatorSubsystem.setSetPointCommand(kMinSafeElevHeight).andThen(
                        Commands.waitUntil(() -> !this.isElevStowed()) //ensures elevator is at a height so it can move
                    ), 
                    Commands.none(), 
                    () -> {return this.isArmStowed() || forceMinSafeHeightMove;}
                ),
                armSubsystem.setSetpointCommand(new Rotation2d(kSafeArmAngle))
            ), 
            Commands.none(),
            () -> {return !this.isElevSafeToMove() || (this.isArmStowed() || forceMinSafeHeightMove);}
        ).andThen(
            Commands.waitUntil(() -> {return this.isElevSafeToMove() && !this.isElevStowed();}).withTimeout(1.0)
        );
    }

    /**
     * Moves the elevator first before moving the arm
     * @return Command to do above actions
     */
    private Command elevatorPriorityMove(DynamicsSetpoint setpoint){
        return Commands.parallel(
            elevatorSubsystem.setSetPointCommand(setpoint.heightMeters),
            Commands.waitUntil(() -> isElevAtSetpoint(setpoint.heightMeters)).withTimeout(2).andThen(
                armSubsystem.setSetpointCommand(setpoint.armAngle)
            )
        );
    }

    /**
     * Moves the arm first before moving the elevator
     * @return Command to do above actions
     */
    private Command armPriorityMove(DynamicsSetpoint setpoint){
        return Commands.parallel(
            armSubsystem.setSetpointCommand(setpoint.armAngle),
            Commands.waitUntil(() -> isArmAtSetpoint(setpoint.armAngle)).withTimeout(2).andThen(
                elevatorSubsystem.setSetPointCommand(setpoint.heightMeters)
            )
        );
    }

    //#endregion

    //#region small Commands

    private Command scoreHeight(DynamicsSetpoint scoringPoint){
        return Commands.sequence(
            makeSystemSafeToMove(false),
            elevatorPriorityMove(scoringPoint)
        );
    }

    private Command loadStow(){
        return Commands.sequence(
            makeSystemSafeToMove(true),
            armPriorityMove(DynaPreset.LOAD.setpoint) //brings arm to the load angle, then drops the elevator
        );
    }

    private Command prescoreStow(){
        return Commands.sequence(
            makeSystemSafeToMove(false),
            armPriorityMove(DynaPreset.PRESCORE.setpoint) //using arm Priority allows the arm to goto the right place then move the elevator down to the needed position 
        );
    }


    //#endregion

    public Command stow(){
        return Commands.either(
            prescoreStow(), 
            loadStow(), 
            this::isCoralInArm
        );
    }

    public Command gotoScore(DynaPreset scorePreset){
        return scoreHeight(scorePreset.setpoint);
    }

    public Command score(){
        return Commands.deadline(
            Commands.waitUntil(
                new Trigger(this::isCoralInArm).negate().debounce(laserCanDebounce)
            ).withTimeout(1.0),
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.OUT)
        ).andThen(intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.NEUTRAL));
    }

    /**
     * this tells the intake to start intaking, it'll end when the funnel Lasercan has detected a coral
     * @return
     */
    public Command blockingIntake(){
        return Commands.deadline(
            Commands.waitUntil(this::funnelDetect).withTimeout(1.0), //TODO remove this timeout outside of sim
            intakeSubsystem.setPresetSpeedCommand(IntakeSpeed.IN)
        );
    }
}