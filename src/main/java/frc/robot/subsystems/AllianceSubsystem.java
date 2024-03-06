package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AllianceSubsystem extends SubsystemBase{

    private boolean isRed;

    /**
     * Initializes subsystem to handle information related to the Field Management System (such as our alliance color).
     */
    public AllianceSubsystem() {

        isRed = false;
    }

    @Override
    public void periodic() {
        
        boolean incomingIsRed = DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);

        if (incomingIsRed != isRed) {
            if (incomingIsRed) {
                System.out.println("Alliance color switched to Red.");
            } else {
                System.out.println("Alliance color switched to Blue.");
            }
        }

        isRed = incomingIsRed;
    }

    /**
     * Identifies whether or not we are Red Alliance.

     * @return isRed boolean
     */
    public boolean isRedAlliance() {

        return isRed;
    }   
}
