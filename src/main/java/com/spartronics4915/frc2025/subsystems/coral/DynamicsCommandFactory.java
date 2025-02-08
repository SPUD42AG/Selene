package com.spartronics4915.frc2025.subsystems.coral;

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
        INTAKEPOSITION(),
        L4SCORE(),
        STOW();
        //add more or positions later

       
       
    }

        switch (moveToRobotPosition) {
        //??maybe???
        }
    

     /* ^^^^^here will be the function that takes in the enum and 
     gives it the info of where it needs to be*/





    
}
