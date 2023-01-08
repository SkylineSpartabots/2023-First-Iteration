package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsSubsystem extends SubsystemBase {
    private static PneumaticsSubsystem instance = null;
    private static PneumaticsSubsystemMode mode;
    
    public static PneumaticsSubsystem getInstance(){
        if(instance == null){
            instance = new PneumaticsSubsystem();
        }
        return instance;
    }

    public PneumaticsSubsystem() {

    }

    public void SetPneumaticsMode(PneumaticsSubsystemMode mode) {
        // TODO: implement solonoids and change power to desired mode
        this.mode = mode;
    }

    @Override
    public String getName() {
        return "PneumaticsSubsystem";
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pneumatics width", this.mode.width);

    }

    public enum PneumaticsSubsystemMode {
        OPEN(10.0),
        CLOSE(0.0),
        CUBE(8.0),
        CONE(5.0);

        private final double width;

        PneumaticsSubsystemMode(double width) {
            this.width = width;
        }

    }

}