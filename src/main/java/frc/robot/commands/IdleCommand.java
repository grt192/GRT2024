package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.climb.ClimbLowerCommand;
import frc.robot.commands.elevator.ElevatorToGroundCommand;
import frc.robot.commands.shooter.flywheel.ShooterFlywheelStopCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollersSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.NotePosition;

public class IdleCommand extends ParallelCommandGroup{
    public IdleCommand(IntakePivotSubsystem intakePivotSubsystem,
                       IntakeRollersSubsystem intakeRollersSubsystem,
                       ElevatorSubsystem elevatorSubsystem,
                       ShooterPivotSubsystem shooterPivotSubsystem,
                       ShooterFlywheelSubsystem shooterFlywheelSubsystem,
                       ClimbSubsystem climbSubsystem,
                       LEDSubsystem ledSubsystem){
        // addRequirements(intakePivotSubsystem, intakeRollersSubsystem,
        //                 elevatorSubsystem,
        //                 shooterPivotSubsystem, shooterFeederSubsystem, shooterFlywheelSubsystem,
        //                 climbSubsystem
        // );

        addCommands(new InstantCommand(() -> intakeRollersSubsystem.setAllRollSpeed(0, 0), intakeRollersSubsystem),
                    new ElevatorToGroundCommand(elevatorSubsystem),
                    new ShooterFlywheelStopCommand(shooterFlywheelSubsystem),
                    new InstantCommand(() -> {
                        if(intakeRollersSubsystem.sensorNow()){
                            ledSubsystem.setNoteMode(NotePosition.INTAKE_HOLDING);
                        } else {
                            ledSubsystem.setNoteMode(NotePosition.NONE);
                        }
                    })//,
                    // new ClimbLowerCommand(climbSubsystem) // NOT USING CLIMB FOR NOW
        );
    }
}
