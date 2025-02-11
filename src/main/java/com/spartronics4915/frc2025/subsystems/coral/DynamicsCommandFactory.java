package com.spartronics4915.frc2025.subsystems.coral;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DynamicsCommandFactory {

    private IntakeSubsystem intakeSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ArmSubsystem armSubsystem;

    public DynamicsCommandFactory(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.armSubsystem = armSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    public enum robotPosition {
        INTAKE_POSITION(Rotation2d.fromDegrees(0), (0)),
        L4_SCORE(Rotation2d.fromDegrees(0), (0)),
        STOW(Rotation2d.fromDegrees(0), (0));

        public Rotation2d angle;
        public double meter;
        //add more or less positions later

        private robotPosition(Rotation2d angle, double meter) {
            this.angle = angle;
            this.meter = meter;
        }
        
        
    }

    public Command goToState(robotPosition newState){
        return Commands.none();
    }

}

    
   







    

