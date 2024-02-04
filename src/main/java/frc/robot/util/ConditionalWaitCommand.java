package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ConditionalWaitCommand extends Command{
    BooleanSupplier op;
    
    public ConditionalWaitCommand(BooleanSupplier condition){
        op = condition;
    }

    @Override
    public boolean isFinished() {
        return op.getAsBoolean();
    }
}
