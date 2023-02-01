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
    private int select = 0;
    int time = 0;
    int gap = 1;

    public void LEDSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();

        //setColor(0);
      }
    /**
   * Sets color of the LED to 
   *
   * 
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
            
    
public int getSelected() {
    return select;
}
    /**
     * @param mode 0:Test 
     * 1: Red Ants 
     * 2: Rainbow
     */
public void setSelected(int selected) {
    
    
    
    this.select = selected;
}


    public void update(int selected) {
            switch (selected) { // uhh what are enums again lol
                case 1: {
                    runAnt();
                    break;
                }

                case 2: {
                    runRainbow();
                    break;
                }

            }
    }

    public void runRainbow() {
        // taste the rainbow (ill do this later aligator)
    }

    public void runAnt(){
        
        
            for (int i = 1; i < m_ledBuffer.getLength(); i++) {
            if(i-gap%5==0) {
                
                m_ledBuffer.setRGB(i, 0, 0, 0);
                
            }
            else{m_ledBuffer.setRGB(i, 255, 0, 0);}

        }
        m_led.setData(m_ledBuffer);
        time = 0;
        if (gap==5) {
            gap=0;
            }
             else {

                gap++;

            };
    }
    

    @Override
	public void periodic() {
        time++; // 1 time = 20 miliseconds pretty sure
        if (time==20) { // updates selected light animation every 400 miliseconds (20*20=400)
            update(getSelected()); 
            time=0;
        }
    
    }
}