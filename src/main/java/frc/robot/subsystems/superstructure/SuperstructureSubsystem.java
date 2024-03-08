package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FieldManagementSubsystem;
import frc.robot.subsystems.leds.LightBarSubsystem;

public class SuperstructureSubsystem extends SubsystemBase {
    
    private final FieldManagementSubsystem fmsSubsystem;
    private final LightBarSubsystem lightBarSubsystem;

    public SuperstructureSubsystem(LightBarSubsystem lightBarSubsystem, FieldManagementSubsystem fmsSubsystem){
        this.fmsSubsystem = fmsSubsystem;
        this.lightBarSubsystem = lightBarSubsystem;
    }

    @Override
    public void periodic() {
        if (fmsSubsystem.getMatchStatus() == MatchStatus.AUTON) {
            lightBarSubsystem.setLightBarStatus(LightBarStatus.AUTON);
        } else if (fmsSubsystem.getMatchStatus() == MatchStatus.ENDGAME) {
            lightBarSubsystem.setLightBarStatus(LightBarStatus.ENDGAME);
        }
    }
}
