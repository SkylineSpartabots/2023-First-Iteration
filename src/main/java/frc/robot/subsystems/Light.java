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

        setColor();
      }

    public void setColor() {

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
         }
         
         m_led.setData(m_ledBuffer);
         
    }
    
}
