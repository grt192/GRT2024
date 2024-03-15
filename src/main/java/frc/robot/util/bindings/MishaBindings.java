package frc.robot.util.bindings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.elevator.ElevatorToAmpCommand;
import frc.robot.commands.elevator.ElevatorToTrapCommand;
import frc.robot.commands.elevator.ElevatorToZeroCommand;
import frc.robot.commands.intake.pivot.IntakePivotSetPositionCommand;
import frc.robot.commands.intake.roller.IntakeRollerIntakeCommand;
import frc.robot.commands.intake.roller.IntakeRollerOuttakeCommand;
import frc.robot.subsystems.climb.ManualClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;

/** Misha's control bindings. */
public class MishaBindings {

    private final XboxController mechController;
    private final LightBarSubsystem lightBarSubsystem;
    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;
    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ManualClimbSubsystem climbSubsystem;

    private final JoystickButton aButton;
    private final JoystickButton bButton;
    private final JoystickButton leftBumper;
    private final JoystickButton rightBumper;
    private final JoystickButton xButton;
    private final JoystickButton yButton;
    private final JoystickButton offsetUpButton;
    private final JoystickButton offsetDownButton;
    private final JoystickButton toggleClimbLimitsButton;

    /** Bindings for Misha. */
    public MishaBindings(
        XboxController mechController, GenericHID switchboard, LightBarSubsystem lightBarSubsystem,
        IntakePivotSubsystem intakePivotSubsystem, IntakeRollerSubsystem intakeRollerSubsystem,
        ElevatorSubsystem elevatorSubsystem,
        ShooterPivotSubsystem shooterPivotSubsystem, ShooterFlywheelSubsystem shooterFlywheelSubsystem,
        ManualClimbSubsystem climbSubsystem
    ) {
        this.mechController = mechController;
        this.lightBarSubsystem = lightBarSubsystem;
        this.intakePivotSubsystem = intakePivotSubsystem;
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.shooterPivotSubsystem = shooterPivotSubsystem;
        this.shooterFlywheelSubsystem = shooterFlywheelSubsystem;
        this.climbSubsystem = climbSubsystem;
        
        aButton = new JoystickButton(mechController, XboxController.Button.kA.value);
        bButton = new JoystickButton(mechController, XboxController.Button.kB.value);
        xButton = new JoystickButton(mechController, XboxController.Button.kX.value);
        yButton = new JoystickButton(mechController, XboxController.Button.kY.value);
        leftBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);
        offsetUpButton = new JoystickButton(switchboard, 7);
        offsetDownButton = new JoystickButton(switchboard, 8);
        toggleClimbLimitsButton = new JoystickButton(switchboard, 9);
    }

    /** Sets all the bindings to this set. */
    public void setAllBindings() {
        bindIntaking();
        bindAmp();
        bindTrap();
        bindShooting();
    }

    /** Binds the intake controls.
     * a runs the intaking sequence
     * b stops the roller in an emergency
     * x toggles the intake pivot, stops the rollers.
     */
    public void bindIntaking() {
        aButton.onTrue(
            new ElevatorToZeroCommand(elevatorSubsystem).andThen(// first lower the elevator (should be down)
                new IntakePivotSetPositionCommand(intakePivotSubsystem, 1).alongWith(// then extend the intake
                    new IntakeRollerIntakeCommand(intakeRollerSubsystem, lightBarSubsystem)
                ).andThen(
                    // intake the note to the color sensor
                    new IntakePivotSetPositionCommand(intakePivotSubsystem, 0) // stow intake
                ).unless(() -> mechController.getLeftTriggerAxis() > .1) // cancel if try to outtake
            )
        );

        bButton.onTrue(new InstantCommand(() -> {}, intakeRollerSubsystem));


        xButton.onTrue(new InstantCommand(() ->  {
            double outPosition = 1;
            double stowPosition = 0;
            if (elevatorSubsystem.getExtensionPercent() > .5 
                && elevatorSubsystem.getTargetState() == ElevatorState.TRAP) {
                outPosition = .38; // push intake out
            } else if (elevatorSubsystem.getExtensionPercent() > .5 
                && elevatorSubsystem.getTargetState() == ElevatorState.AMP) {
                stowPosition = .2;
            }

            intakePivotSubsystem.setPosition(
                intakePivotSubsystem.getEncoderPosition() < (outPosition + stowPosition) / 2 
                ? outPosition 
                : stowPosition
            );

        }, intakePivotSubsystem).alongWith(new InstantCommand(() -> {}, intakeRollerSubsystem)));

        intakeRollerSubsystem.setDefaultCommand(new InstantCommand(() -> {

            double power = .7 * (mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis());

            intakeRollerSubsystem.setRollSpeeds(power, power);
        }, intakeRollerSubsystem));
        
    }

    /** Binds for the amp controls.
     * Amp sequence is:
     * 1. right bumper feeds note to correct position and raises elevator
     * 2. left trigger shoots
     */
    public void bindAmp() {
        rightBumper.onTrue(
            new ConditionalCommand(
                    // if elevator is up
                    new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(// lower the elevator
                        () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem)), // stow the pivot
                    // if elevator is down
                    new IntakePivotSetPositionCommand(intakePivotSubsystem, 1).andThen(// extend pivot
                        new IntakeRollerOuttakeCommand(intakeRollerSubsystem, .75) // run rollers to front sensor
                                .until(() -> intakeRollerSubsystem.getFrontSensorValue() > .12),
                        new ElevatorToAmpCommand(elevatorSubsystem), // raise elevator
                        new IntakePivotSetPositionCommand(intakePivotSubsystem, 0.2) // angle intake for scoring
                    ).until(() -> mechController.getLeftTriggerAxis() > .05 
                        || mechController.getRightTriggerAxis() > .05
                    ), 
                    // check if the elevator is currently targeting one of the upper positions to choose what to do
                    () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP
                        || elevatorSubsystem.getTargetState() == ElevatorState.TRAP));
        
    }

    /** Binds the trap controls.
     * Trap sequence is: 
     * 1. left bumper to feed note to front
     * 2. joysticks to raise climb
     * 3. left bumper to raise elevator once past chain
     * 4. joysticks down to climb
     * 6. left trigger to shoot note into trap
     */
    public void bindTrap() {

        climbSubsystem.setDefaultCommand(new RunCommand(() -> {
            climbSubsystem.setSpeeds(-mechController.getLeftY(), -mechController.getRightY());
        }, climbSubsystem));

        toggleClimbLimitsButton.onTrue(new InstantCommand(() -> climbSubsystem.enableSoftLimits(false)));
        toggleClimbLimitsButton.onFalse(new InstantCommand(() -> climbSubsystem.enableSoftLimits(true)));

        leftBumper.onTrue(
            new ConditionalCommand(
                new ElevatorToZeroCommand(elevatorSubsystem).alongWith(new InstantCommand(// lower the elevator
                    () -> intakePivotSubsystem.setPosition(0), intakePivotSubsystem)), // stow intake
                new ConditionalCommand(
                    new IntakePivotSetPositionCommand(intakePivotSubsystem, .38).andThen(
                        new ElevatorToTrapCommand(elevatorSubsystem)
                    ),
                    new IntakePivotSetPositionCommand(intakePivotSubsystem, 1).andThen(// extend pivot
                        new IntakeRollerOuttakeCommand(intakeRollerSubsystem, .75) // run rollers to front sensor
                                .until(() -> intakeRollerSubsystem.getFrontSensorValue() > .12),
                        new IntakePivotSetPositionCommand(intakePivotSubsystem, 0)
                    ),
                    intakeRollerSubsystem::getFrontSensorReached
                ), // raise the elevator
                () -> elevatorSubsystem.getTargetState() == ElevatorState.AMP // check if targeting a high pos
                    || elevatorSubsystem.getTargetState() == ElevatorState.TRAP)
        );
        
    }

    /** Binds the controls for shooting notes.
     * Shoot sequence is:
     * 1. y hold to spin up wheel
     * 2. right trigger to shoot
     */
    public void bindShooting() {

        shooterFlywheelSubsystem.setDefaultCommand(new InstantCommand(() -> {
            if (yButton.getAsBoolean()) {
                lightBarSubsystem.setLightBarStatus(LightBarStatus.SHOOTER_SPIN_UP);
                // shooterFlywheelSubsystem.setShooterMotorSpeed(shooterTopSpeed, shooterBotSpeed); // for tuning
                shooterFlywheelSubsystem.setShooterMotorSpeed();
                shooterPivotSubsystem.setAutoAimBoolean(true);
            } else {
                shooterPivotSubsystem.setAutoAimBoolean(false);
                shooterFlywheelSubsystem.stopShooter();
            }

            // if we are at speed, rumble the mech controller
            if (shooterFlywheelSubsystem.atSpeed()) {
                mechController.setRumble(RumbleType.kBothRumble, .4);
            } else {
                mechController.setRumble(RumbleType.kBothRumble, 0);
                if (lightBarSubsystem.getLightBarStatus() == LightBarStatus.SHOOTER_SPIN_UP) {
                    double top = shooterFlywheelSubsystem.getTopSpeed() / shooterFlywheelSubsystem.getTargetTopRPS();
                    double bottom = shooterFlywheelSubsystem.getBottomSpeed() 
                                  / shooterFlywheelSubsystem.getTargetBottomRPS();
                    double avg = (top + bottom) / 2; // in case they're different, this just shows the average. 

                    lightBarSubsystem.updateShooterSpeedPercentage(avg);
                }
                
            }
        }, shooterFlywheelSubsystem));

        // Offset buttons to correct the shooter if needed
        offsetUpButton.onTrue(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(3)))
        );
        offsetUpButton.onFalse(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0)))
        );

        offsetDownButton.onTrue(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(-3)))
        );
        offsetDownButton.onFalse(new InstantCommand(
            () -> shooterPivotSubsystem.setAngleOffset(Units.degreesToRadians(0)))
        );
    }
  

}
