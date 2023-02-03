package frc.robot.subsystems;

import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/*setRGB actually takes RBG inputs not RGB G & B are flipped */

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
    private int selected = 1;
    private int time = 0;
    int gap = 1; // subtracted from LED index to get it to be evenly divisible by group size
    int limiter = 13; // limits how often the update() method is called
    int groupSize = 0; // changes when gap resets - limit gap - how big in between ants

    public void Light() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void increaseTime() {
        time++;
    }

    public void decreaseTime() {
        if(time>0) {
            time--;
        }
    }

    public void setSolidColor(int r, int g, int b){
        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, r, g, b);
            m_led.setData(m_ledBuffer);
        }
    }

    public int getSelected() {
        return selected;
    }
    /**
     * @param selected 
     * 0: Test 
     * 1: Red Ants 
     * 2: Rainbow Solid
     * 3: Rainbow Segmented
     * 4: Caution
     * 5: Velocity
     */
    public void setSelected(int selected) {
        this.selected = selected;
    }

    public void update(int selected) {
            switch (selected) { // uhh what are enums again lol
                case 0:{
                    setSolidColor(255, 255, 255);
                    m_led.setData(m_ledBuffer);
                    break;}
                    
                case 1: {
                    groupSize= 5;
                    runAnt();
                    break;}

                case 2: {
                    runRainbowSolid();
                    break;
                }

                case 3: {
                    groupSize = 6;
                    runSegmentedRainbow();
                    break;
                }
                case 4: {
                    groupSize = 3;
                    runCaution();
                    break;
                }
                case 5: {
                    runVelocity();
                    break;
                }
            }
    }

    public void runSegmentedRainbow() {
        
        for (int i = 1; i < m_ledBuffer.getLength(); i++) {
            if((i-gap)%1==0) {m_ledBuffer.setRGB(i, 255, 0, 0);}
            else if((i-gap)%2==0) {m_ledBuffer.setRGB(i, 255, 0, 127);}

            else if((i-gap)%3==0) {m_ledBuffer.setRGB(i, 255, 0, 255);}

            else if((i-gap)%4==0) {m_ledBuffer.setRGB(i, 0, 0, 255);}

            else if((i-gap)%5==0) {m_ledBuffer.setRGB(i, 0, 255, 0);}

            else if((i-gap)%6==0) {m_ledBuffer.setRGB(i, 75, 130, 0);} 

            else {m_ledBuffer.setRGB(i, 148, 211, 0);}

        }
        m_led.setData(m_ledBuffer);

        gapReset();
        // time = 0;
        // if (gap==groupSize) {
        //     gap=0;
        // } 
        // else {
        //     gap++;
        // };

    };

    public void runVelocity() { //uses accelrometer
        double averageVelocity = 0.0;
        for (SwerveModule mod : Swerve.getInstance().getSwerveModules()) {
            averageVelocity += mod.getState().speedMetersPerSecond/4;
        }
    }

    public void runRainbowSolid() {
        for (int h =0; h < 180; h++){
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, h, 255, 255);
                m_led.setData(m_ledBuffer);
            }
        }
        
        
   }

    public void runAnt(){
        
        for (int i = 1; i < m_ledBuffer.getLength(); i++) {
            if((i-gap)%5==0) { // could this 5 be replaced with groupSize???
                
                m_ledBuffer.setRGB(i, 255, 0, 0);
                
            }
            else{m_ledBuffer.setRGB(i, 0, 0, 0);}

        }
        m_led.setData(m_ledBuffer);

        gapReset();
        // time = 0;
        // if (gap==groupSize) {
        //     gap=0;
        // } 
        // else {
        //     gap++;
        // };
    }
    
    public void runCaution(){ //play when robot slows down?
        for (int i = 1; i < m_ledBuffer.getLength(); i++) {
            if((i-gap)%2==0) {

                m_ledBuffer.setRGB(i, 255, 0, 210);}
                
            else{m_ledBuffer.setRGB(i, 255, 0, 0);}

        }
        m_led.setData(m_ledBuffer);

        gapReset();
    }

    public void gapReset(){
        time = 0;
        if (gap==groupSize) {
            gap=0;
        } 
        else {
            gap++;
        };
    }

    @Override
	public void periodic() {

        SmartDashboard.putNumber("LED mode", getSelected());

        time++; // 1 time = 20 miliseconds pretty sure
        if (time== limiter) { // updates selected light animation (20*time = miliseconds)
            update(getSelected()); 
            time=0;
        }
    }
}