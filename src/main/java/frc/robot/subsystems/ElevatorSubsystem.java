package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    static ElevatorSubsystem instance;
    public ElevatorSubsystem getInstance() {
        if (instance == null) instance = new ElevatorSubsystem();
        return instance;
    }
    public ElevatorSubsystem() {
        
    }
}
