package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ElevatorToAMPCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;
    private NetworkTableInstance nt;
    private NetworkTable table;
    private NetworkTableEntry entry;
    public ElevatorToAMPCommand(ElevatorSubsystem elevatorSubsystem){
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted){
        System.out.println("TO AMP FINISHED");
        if (interrupted){
            System.out.println("TO AMP INTERUPTED");
        }
        nt = NetworkTableInstance.getDefault();
        table = nt.getTable("elevator");
        entry = table.getEntry("position");
        entry.setInteger(3);
        return;
                
         
    }

    @Override
    public void execute(){
        this.elevatorSubsystem.setTargetState(ElevatorState.AMP);
    }

    @Override
    public boolean isFinished(){
        return elevatorSubsystem.atState(ElevatorState.AMP);
    }
}