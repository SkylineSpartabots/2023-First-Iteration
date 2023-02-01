package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Light extends SubsystemBase {
    static Light instance;

    public static Light getInstance() {
        if (instance == null) {
            instance = new Light();
        }
        return instance;
    }

    public AddressableLED m_led = new AddressableLED(Constants.LEDConstants.ledPin);
    public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.ledBufferSize);


    public void LEDSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        //setColor(0);
      }
    /**
   * Sets color of the LED to 
   *
   * @param mode 0:Test 1:Solid Red 2:Solid Green 3:Solid Blue 4:Ant Trail 5:Rainbow 6:
   */
    // public void setColor(int mode) {
        
    //     for (var i = 0; i < m_ledBuffer.getLength(); i++) {
    //         // Sets the specified LED to the RGB values for green

    //         switch (mode) {
    //             case 0: //Test
    //                 m_ledBuffer.setRGB(i, 0,255,0);
    //                 break;
    //             case 1: //Reset Odometry
    //                 m_ledBuffer.setRGB(i, 255,0,0);
    //                 break;

    //             case 2: //Zero Gyro
    //                 m_ledBuffer.setRGB(i,0,0,255);
    //                 break;
    //             default:
    //                 break;
    //          }
            
    //         m_led.setData(m_ledBuffer);
    //      }
    // }
    public void runAnt(){
        
        for (int k = 1; k < 5; k++) {
            for (int i = 1; i < m_ledBuffer.getLength(); i++) {
            if(i-k%5==0) {
                
                m_ledBuffer.setRGB(i, 0, 0, 0);
                
            }
            else{m_ledBuffer.setRGB(i, 255, 0, 0);}

        }
        m_led.setData(m_ledBuffer);
        }
        
    }
    

    @Override
	public void periodic() {
        runAnt();
    
    }
}