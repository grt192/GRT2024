package frc.robot.util.bindings;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.shooter.pivot.ShooterPivotSetAngleCommand;
import frc.robot.subsystems.climb.ManualClimbSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakePivotSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;
import frc.robot.subsystems.shooter.ShooterFlywheelSubsystem;
import frc.robot.subsystems.shooter.ShooterPivotSubsystem;
import frc.robot.subsystems.superstructure.LightBarStatus;
import frc.robot.util.GRTUtil;

/** Bindings to test subsystems. */
public class TestBindings {
    private final XboxController mechController;
    private final LightBarSubsystem lightBarSubsystem;
    private final IntakePivotSubsystem intakePivotSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ShooterPivotSubsystem shooterPivotSubsystem;
    private final ShooterFlywheelSubsystem shooterFlywheelSubsystem;
    private final ManualClimbSubsystem climbSubsystem;

    private final JoystickButton leftBumper;
    private final JoystickButton rightBumper;
    private final JoystickButton yButton;

    private double shooterPivotSetPosition = Units.degreesToRadians(18);
    private double shooterTopSpeed = .7;
    private double shooterBotSpeed = .5;
    private double intakePosition = 0;

    /** Bindings for Misha. */
    public TestBindings(
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
        
        leftBumper = new JoystickButton(mechController, XboxController.Button.kLeftBumper.value);
        rightBumper = new JoystickButton(mechController, XboxController.Button.kRightBumper.value);
        yButton = new JoystickButton(mechController, XboxController.Button.kY.value);
    }

    /** Sets all of the test bindings. */
    public void setAllBindings() {
        bindIntake();
        bindElevator();
        bindShooting(false);
        bindClimb();
    }

    /** Binds test controls for the intake. */
    public void bindIntake() {
        intakePivotSubsystem.setDefaultCommand(new InstantCommand(() -> {
            switch (mechController.getPOV()) {
                case 90:
                    intakePosition += .01;
                    break;
                
                case 270:
                    intakePosition -= .01;
                    break;
                
                default:
                    break;
            }
        }, intakePivotSubsystem));

        intakeRollerSubsystem.setDefaultCommand(new InstantCommand(() -> {

            double power = .7 * (mechController.getRightTriggerAxis() - mechController.getLeftTriggerAxis());

            intakeRollerSubsystem.setRollSpeeds(power, power);
        }, intakeRollerSubsystem));
    }

    /** Binds the test controls for the elevator. */
    public void bindElevator() {
        elevatorSubsystem.setManual();
        elevatorSubsystem.setDefaultCommand(new InstantCommand(() ->
            elevatorSubsystem.setManualPower(mechController.getRightX()),
            elevatorSubsystem)
        );
    }

    

    /** Bind the test controls for the shooter. */
    public void bindShooting(boolean setAutoPivot) {
        shooterPivotSubsystem.setDefaultCommand(new InstantCommand(() -> {
            //TODO: switch to enum for dpad angles

            switch (mechController.getPOV()) {
                case 0:
                    shooterPivotSetPosition += .003;
                    break;

                case 180:
                    shooterPivotSetPosition -= .003;
                    break;

                case 45:
                    shooterTopSpeed += .001;
                    break;

                case 315:
                    shooterTopSpeed -= .001;
                    break;

                case 135:
                    shooterBotSpeed += .001;
                    break;

                case 225:
                    shooterBotSpeed -= .001;
                    break;

                default:
                    break;
            }

            System.out.println(intakePosition);


            
            System.out.print(" Top: " + GRTUtil.twoDecimals(shooterTopSpeed)
                           + " Bot: " + GRTUtil.twoDecimals(shooterBotSpeed)
            );
            shooterPivotSubsystem.printAutoAimInfo();

            shooterPivotSubsystem.setAutoAimBoolean(setAutoPivot);
            if (!setAutoPivot) {
                shooterPivotSubsystem.setAngle(shooterPivotSetPosition);
            }

            if (yButton.getAsBoolean()) {
                lightBarSubsystem.setLightBarStatus(LightBarStatus.SHOOTER_SPIN_UP);
                shooterFlywheelSubsystem.setShooterMotorSpeed(shooterTopSpeed, shooterBotSpeed);
            } else {
                shooterFlywheelSubsystem.stopShooter();
            }
        }, shooterPivotSubsystem));

        rightBumper.onTrue(new ShooterPivotSetAngleCommand(shooterPivotSubsystem,
            Units.degreesToRadians(18)));

        leftBumper.onTrue(new ShooterPivotSetAngleCommand(shooterPivotSubsystem,
            Units.degreesToRadians(60)));
    }

    /** Bind the test controls for climb. */
    public void bindClimb() {
        climbSubsystem.setDefaultCommand(new RunCommand(() -> {
            climbSubsystem.setSpeeds(-mechController.getLeftY(), -mechController.getRightY());
        }, climbSubsystem));
    }


    
}
