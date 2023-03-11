package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class NewIntake {
    
    private static NewIntake instance;
    CANSparkMax m_leader, m_follower;

    public static NewIntake getInstance() {
        if (instance == null) {
            instance = new NewIntake();
        }
        return instance;
    }

    NewIntake() {
        m_leader = new CANSparkMax(-69, MotorType.kBrushless);
        m_follower = new CANSparkMax(-6969, MotorType.kBrushless);
    }


}
