package frc.robot.commands.elevator;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/** Sends the elevator to position zero. */
public class ElevatorToZeroCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    private NetworkTableInstance nt;
    private NetworkTable table;
    private NetworkTableEntry entry;
    
    /** Constructs a {@link ElevatorToZeroCommand} for the specified elevator. */
    public ElevatorToZeroCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("TO GROUND FINISHED");
        if (interrupted) {
            System.out.println("TO GROUND INTERRUPTED");
        }

        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("elevator");
        entry = table.getEntry("position");
        entry.setInteger(0);
        
        return;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        elevatorSubsystem.setTargetState(ElevatorState.ZERO);
    }

    @Override
    public boolean isFinished() {
        System.out.println("TO GROUND RUNNING");
        return elevatorSubsystem.atGround();
    }
}