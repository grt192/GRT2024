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
import frc.robot.Constants.AutoAlignConstants;
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

    private double targetTopRPS;
    private double targetBottomRPS;

    private Pose2dSupplier poseSupplier;

    private AkimaSplineInterpolator akima;
    private PolynomialSplineFunction topFlywheelSpline;
    private PolynomialSplineFunction bottomFlywheelSpline;
    private boolean atSpeed = false;

    public ShooterFlywheelSubsystem(Pose2dSupplier poseSupplier){
        //motors
        shooterMotorTop = new TalonFX(ShooterConstants.SHOOTER_MOTOR_TOP_ID);
        shooterMotorBottom = new TalonFX(ShooterConstants.SHOOTER_MOTOR_BOTTOM_ID);

        shooterMotorTop.setInverted(true);
        shooterMotorBottom.setInverted(true);

        shooterMotorTop.setNeutralMode(NeutralModeValue.Coast);
        shooterMotorBottom.setNeutralMode(NeutralModeValue.Coast);

        Slot0Configs configs = new Slot0Configs();

        double[] distances = {ShooterConstants.MIN_SHOOTER_DISTANCE, 2, 3, 3.71, 4.8, 5.6, ShooterConstants.MAX_SHOOTER_DISTANCE}; //TODO: 1.2 as lowest

        double[] topSpeeds = {.4, .5, .7, .75, .75, .75, .75};
        double[] bottomSpeeds = {.5, .5, .35, .4, .4, .75, .75};

        targetTopRPS = 0.0;
        targetBottomRPS = 0.0; 

        akima = new AkimaSplineInterpolator();
        topFlywheelSpline = akima.interpolate(distances, topSpeeds);
        bottomFlywheelSpline = akima.interpolate(distances, bottomSpeeds);

        configs.kP = .5;
        configs.kI = 0.005;
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
        targetTopRPS = ShooterConstants.MAX_FLYWHEEL_RPS * topSpeed;
        targetBottomRPS = ShooterConstants.MAX_FLYWHEEL_RPS * bottomSpeed;

        // System.out.println("TARGET RPS " + targetBottomRPS + " CURRENT " + shooterMotorBottom.getVelocity().getValueAsDouble());

        shooterMotorTop.setControl(request.withVelocity(targetTopRPS));
        shooterMotorBottom.setControl(request.withVelocity(targetBottomRPS));
        
        atSpeed = Math.abs(targetTopRPS - shooterMotorTop.getVelocity().getValueAsDouble()) < 5
            && Math.abs(targetBottomRPS - shooterMotorBottom.getVelocity().getValueAsDouble()) < 5
            && targetBottomRPS != 0;
        // System.out.println("shooter motor speed is: " + shooterMotorTop.get());
    }

    /** Sets shooting motor speed for only one speed. */
    public void setShooterMotorSpeed(double speed) {
        setShooterMotorSpeed(speed, speed);
    }

    public void setShooterMotorSpeed(){
        setShooterMotorSpeed(getTopSpeed(), getBottomSpeed());
    }

    public void stopShooter(){
        shooterMotorTop.set(0);
        shooterMotorBottom.set(0);
        atSpeed = false;
    }

    public boolean atSpeed() {
        return atSpeed;
    }

    public double getTopSpeed() {
        return topFlywheelSpline.value(getShootingDistance());
    }

    public double getBottomSpeed() {
        return bottomFlywheelSpline.value(getShootingDistance());
    }

    /**
     * Gets the actual speed of the top shooter motor.
     *
     * @return the actual speed in rotations per second
     */
    public double getActualTopSpeed() {
        return shooterMotorTop.getVelocity().getValueAsDouble();
    }
    
    /**
     * Gets the actual speed of the bottom shooter motor.
     *
     * @return the actual speed in rotations per second
     */
    public double getActualBottomSpeed() {
        return shooterMotorBottom.getVelocity().getValueAsDouble();
    }

    /**
     * Gets the setpoint speed of the top shooter motor.
     *
     * @return the target speed in rotations per second
     */
    public double getTargetTopRPS() {
        return targetTopRPS;
    }

    /**
     * Gets the setpoint speed of the bottom shooter motor.
     *
     * @return the target speed in rotations per second
     */
    public double getTargetBottomRPS() {
        return targetBottomRPS;
    }

    private double getShootingDistance() {
        double currentDistance;
        double speakerHeight = Units.inchesToMeters(80.51);
        Pose2d currentField = poseSupplier.getPose2d();
        //System.out.println("Angle of shooter" + Math.atan(speakerHeight/distance));

        if (SwerveConstants.IS_RED) {  //true = red
            double xLength = Math.pow(currentField.getX() - AutoAlignConstants.RED_SPEAKER_POSE.getX(), 2);
            double yLength = Math.pow(currentField.getY() - AutoAlignConstants.RED_SPEAKER_POSE.getY(), 2);
            currentDistance = Math.sqrt(xLength + yLength);

        } else {
            double xLength = Math.pow(currentField.getX() - AutoAlignConstants.BLUE_SPEAKER_POSE.getX(), 2);
            double yLength = Math.pow(currentField.getY() - AutoAlignConstants.BLUE_SPEAKER_POSE.getY(), 2);

            currentDistance = Math.sqrt(xLength + yLength);
        }

        return MathUtil.clamp(currentDistance,
            ShooterConstants.MIN_SHOOTER_DISTANCE, ShooterConstants.MAX_SHOOTER_DISTANCE
        );
    }

    @Override
    public void periodic() {
        ntPublisher.set(atSpeed());
    }
}