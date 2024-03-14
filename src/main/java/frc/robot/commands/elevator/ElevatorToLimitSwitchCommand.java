package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sends the elevator to position zero. */
public class ElevatorToLimitSwitchCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    
    /** Constructs a {@link ElevatorToLimitSwitchCommand} for the specified elevator. */
    public ElevatorToLimitSwitchCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setMotorPower(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setMotorPower(0);
        elevatorSubsystem.zeroEncoder();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getLimitSwitch();
    }
}