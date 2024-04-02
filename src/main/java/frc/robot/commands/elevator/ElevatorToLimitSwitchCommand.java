package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sends the elevator to position zero. */
public class ElevatorToLimitSwitchCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private boolean isFinished = false; 
    /** Constructs a {@link ElevatorToLimitSwitchCommand} for the specified elevator. */
    public ElevatorToLimitSwitchCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        try {
            if (!elevatorSubsystem.getLimitSwitch()){
                elevatorSubsystem.setCoast();
                elevatorSubsystem.setMotorPower(0);
            }
        }
        catch (Exception e) {
            System.out.print(e);
            isFinished = true;
            return;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setBrake();
        System.out.print("0 motor");
        elevatorSubsystem.zeroEncoder();
    }

    @Override
    public boolean isFinished() {
        try{
            isFinished = elevatorSubsystem.getLimitSwitch();
        }
        catch (Exception e) {
            System.out.print(e);
            isFinished = true;
        }
        return isFinished;
    }
}