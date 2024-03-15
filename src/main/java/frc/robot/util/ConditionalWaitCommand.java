package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Waits until a condition is met. */
public class ConditionalWaitCommand extends Command {
    BooleanSupplier op;
    
    /** Waits until a condition is met. */
    public ConditionalWaitCommand(BooleanSupplier condition) {
        op = condition;
    }

    @Override
    public boolean isFinished() {
        return op.getAsBoolean();
    }
}
