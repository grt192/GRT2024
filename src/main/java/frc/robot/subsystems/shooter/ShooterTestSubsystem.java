package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;

public class ShooterTestSubsystem extends SubsystemBase{
    
    private final XboxController controller;

    private final CANSparkMax topmotor;
    private final CANSparkMax bottommotor;

    private double topMotorVel;
    private double botMotorVel;

    public ShooterTestSubsystem(){
        
        controller = new XboxController(0);

        topmotor = new CANSparkMax(1, MotorType.kBrushless);
        bottommotor = new CANSparkMax(2, MotorType.kBrushless);

        topMotorVel = 0.0;
        botMotorVel = 0.0;
    }
    
    public void periodic(){

        if(controller.getRightBumperPressed()){
            topMotorVel += 0.01;
            botMotorVel += 0.01;
        }

        if(controller.getLeftBumperPressed()){
            topMotorVel -= 0.01;
            botMotorVel -= 0.01;
        }

        if(controller.getAButtonPressed()){
            topMotorVel += 0.01;
        }

        if(controller.getBButtonPressed()){
            topMotorVel -= 0.01;
        }

        if(controller.getXButtonPressed()){
            botMotorVel += 0.01;
        }

        if(controller.getYButtonPressed()){
            botMotorVel -= 0.01;
        }

        System.out.println("TOP: " + topMotorVel);
        System.out.println("BOTTOM: " + botMotorVel);
        topmotor.set(topMotorVel);
        bottommotor.set(botMotorVel);
    }

}