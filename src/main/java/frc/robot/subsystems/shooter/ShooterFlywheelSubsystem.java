// INTAKE one talon for feeding one neo for  pivot
// SHOOTER one neo for shooting, one neo for pivot, one talon for conveyer belt

//create enums, expecting, holding, firing, and no note

package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.util.Pose2dSupplier;

import org.apache.commons.math3.analysis.interpolation.AkimaSplineInterpolator;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

/** Inits motors and state enums for shooter subsystem. */
public class ShooterFlywheelSubsystem extends SubsystemBase {

    //motors
    private final TalonFX shooterMotorTop;
    private final TalonFX shooterMotorBottom;

    //devices
    private ShooterState currentState; //an enum state thing
    private VelocityVoltage request = new VelocityVoltage(0).withSlot(0);

    //Network Tables
    private NetworkTableInstance ntInstance;
    private NetworkTable ntTable;
    private BooleanPublisher ntPublisher;
    /** Motors assigned. */

    private double topSpeed;
    private double bottomSpeed;

    private Pose2dSupplier poseSupplier;

    private AkimaSplineInterpolator akima;
    private PolynomialSplineFunction topFlywheelSpline;
    private PolynomialSplineFunction bottomFlywheelSpline;
    public ShooterFlywheelSubsystem(Pose2dSupplier poseSupplier){
        //motors
        shooterMotorTop = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ID);
        shooterMotorBottom = new TalonFX(ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID);

        shooterMotorTop.setInverted(true);
        shooterMotorBottom.setInverted(true);

        shooterMotorTop.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorBottom.setNeutralMode(NeutralModeValue.Coast);

        Slot0Configs configs = new Slot0Configs();

        double[] distances = {1.08, 2, 3, 4, 5, 6, 7, 8, 9, 10};

        double[] topSpeeds = {.4, .5, .57, 1, 1, 1, 1, 1, 1, 1};
        double[] bottomSpeeds = {.5, .5, .57, 1, 1, 1, 1, 1, 1, 1};

        akima = new AkimaSplineInterpolator();
        topFlywheelSpline = akima.interpolate(distances, topSpeeds);
        bottomFlywheelSpline = akima.interpolate(distances, bottomSpeeds);

        configs.kP = .4;
        configs.kI = 0;
        configs.kD = 0;
        configs.kV = .12;

        shooterMotorBottom.getConfigurator().apply(configs);
        shooterMotorTop.getConfigurator().apply(configs);

        this.poseSupplier = poseSupplier;
        //nts
        ntInstance = NetworkTableInstance.getDefault();
        ntTable = ntInstance.getTable("RobotStatus");
        ntPublisher = ntTable.getBooleanTopic("shooterReady").publish();
    }

    /** Gets current state of shooter. */
    public ShooterState getShooterState(ShooterState state) {
        return currentState;
    }

    /** Sets current state of shooter. */
    public void setShooterState(ShooterState newState) {
        currentState = newState;
    }

    /** Sets shooting motor speed.  */
    public void setShooterMotorSpeed(double topSpeed, double bottomSpeed) {
        double targetTopRPS = ShooterConstants.MAX_FLYWHEEL_RPS * topSpeed;
        double targetBottomRPS = ShooterConstants.MAX_FLYWHEEL_RPS * bottomSpeed;

        // System.out.println("TARGET RPS " + targetTopRPS + " CURRENT " + shooterMotorTop.getVelocity());

        shooterMotorTop.setControl(request.withVelocity(targetTopRPS));
        shooterMotorBottom.setControl(request.withVelocity(targetBottomRPS));

        //System.out.println("shooter motor speed is: " + shooterMotorTop.get());
    }

    /** Sets shooting motor speed for only one speed. */
    public void setShooterMotorSpeed(double speed) {
        setShooterMotorSpeed(speed, speed);
    }

    public void setShooterMotorSpeed(){
        setShooterMotorSpeed(getTopSpeed(), getBottomSpeed());
    }

    public boolean atSpeed() {
        return shooterMotorTop.getClosedLoopError().getValueAsDouble() < 10.0
            && shooterMotorBottom.getClosedLoopError().getValueAsDouble() < 10.0
            && Math.abs(shooterMotorTop.get()) > .01;
    }

    public double getTopSpeed() {
        return topFlywheelSpline.value(getShootingDistance());
    }

    public double getBottomSpeed() {
        return bottomFlywheelSpline.value(getShootingDistance());
    }

    private double getShootingDistance() {
        double currentDistance;
        double speakerHeight = Units.inchesToMeters(80.51);
        Pose2d currentField = poseSupplier.getPose2d();
        //System.out.println("Angle of shooter" + Math.atan(speakerHeight/distance));

        if (SwerveConstants.IS_RED) {  //true = red
            double xLength = Math.pow(currentField.getX() - ShooterConstants.RED_X, 2);
            double yLength = Math.pow(currentField.getY() - ShooterConstants.RED_Y, 2);
            currentDistance = Math.sqrt(xLength + yLength);

        } else {
            double xLength = Math.pow(currentField.getX() - ShooterConstants.BLUE_X, 2);
            double yLength = Math.pow(currentField.getY() - ShooterConstants.BLUE_Y, 2);

            currentDistance = Math.sqrt(xLength + yLength);
        }

        return MathUtil.clamp(currentDistance, ShooterConstants.MIN_SHOOTER_DISTANCE, ShooterConstants.MAX_SHOOTER_DISTANCE);
    }

    @Override
    public void periodic() {
        ntPublisher.set(atSpeed());
    }
}