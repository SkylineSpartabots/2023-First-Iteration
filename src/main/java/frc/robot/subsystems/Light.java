package frc.robot.subsystems;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

import java.util.Random;
import java.math.*;

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

    private final Intake s_Intake = Intake.getInstance();

    public AddressableLED m_led = new AddressableLED(Constants.LEDConstants.ledPin);
    public AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.ledBufferSize);
    Random rand = new Random();
    private int selected = 1;
    private int time = 0;
    int gap = 1; // subtracted from LED index to get it to be evenly divisible by group size
    int limiter = 13; // limits how often the update() method is called
    int groupSize = 0; // changes when gap resets - limit gap - how big in between ants
    int minusGap = 0; // gap but it goes down not up lol
    int[] randColor = {rand.nextInt(1,255), rand.nextInt(1,255), rand.nextInt(1,255)};
    int finish = 0;
    boolean grabbed = false; // added a thing in intake to set it to off when reversed.

    // varibles needed for valocity
    double p = 0;

    public void Light() { //inizilize the leds. yes this will be a warning forever
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
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

    public void newRandomColor() {
        for (int i = 0; i < randColor.length; i++) {
            randColor[1] = rand.nextInt(10,255);
        }
    }

    public void increaseTime() {
        time++;
    }

    public void decreaseTime() {
        if(time>0) {
            time--;
        }
    }

    public int getSelected() {
        return selected;
    }
    /**
     * @param selected 
     * 0: runWave
     * 1: runAnt
     * 2: runRainbowSolid
     * 3: runSegmentedRainbow
     * 4: runVelocity
     * Modes with triggers:
     * 5: runCaution
     * 6: runGrab
     */
    public void setSelected(int selected) {
        this.selected = selected;
    }

    public void setRandomNormal() {
        selected = rand.nextInt(1, 5);
    }

    public void setRandomRelaxed(){
        // iggy what does this do?
    }

    public void update(int selected) {
            switch (selected) { // uhh what are enums again lol
                case 0:{
                    limiter = 1;
                    runWave();
                    break;
                }
                case 1: {
                    groupSize= 5;
                    limiter = 8;
                    runAnt();
                    break;
                }
                case 2: {
                    limiter = 2;
                    runRainbowSolid();
                    break;
                }
                case 3: {
                    groupSize = 6;
                    limiter = 4;
                    runSegmentedRainbow();
                    break;
                }
                case 4: {
                    limiter = 1;
                    runVelocity();
                    break;
                }
                case 5: {
                    groupSize = 3;
                    limiter = 7;
                    runCaution();
                    break;
                }
                case 6: {
                    groupSize = 3;  
                    limiter = 8;
                    runGrab();
                    break;
                }
                case 7: {
                    
                    break;
                }
            }
    }
    /*Begin LED mode methods */
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
        //     
        // } 
        // else {
        //     gap++;
        // };
    }

    public void runVelocity() { //uses accelrometer
        double averageVelocity = 0.0;
        for (SwerveModule mod : Swerve.getInstance().getSwerveModules()) {
            averageVelocity += mod.getState().speedMetersPerSecond/4;
        }
        // 4.572 mps max
        double d = averageVelocity - p;

        if (d>=0.1) {p+=0.2;} 
        else if (d<=-0.2) {p-=0.2;}

        int color = (int)Math.round(p/4.572);
        color = color*240; //240 is a value color

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setHSV(i, color, 255, 255);
        }

    }

    public void runWave() {
        for (int i = 50; i < m_ledBuffer.getLength()/2; i++) {
            if(i==gap) { 

                m_ledBuffer.setRGB(i, randColor[0], randColor[1], randColor[2]);
            }

        }
        gapReset();
        for (int i = m_ledBuffer.getLength()/2; i > 0; i--) {
            if(i==minusGap) { 

                m_ledBuffer.setRGB(i, randColor[0], randColor[1], randColor[2]);
            }

        }

        if (minusGap==0) {
            minusGap = 50;
            newRandomColor();
        } 
        else {
            gap--;
        };
        
    }

    public void runRainbowSolid() {
        for (int h =0; h < 180; h++){
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setHSV(i, h, 255, 255);
            }
            m_led.setData(m_ledBuffer);
        }
   }

   public void runGrab() { 
    if (s_Intake.getVolts() >= 14) { grabbed = true;} // I got the voltage but idk how much it would jump. 14 is a guess cause the motor opperates at 12v (i think) so 14v is >
    
    if (grabbed) { // is this when it does have something? cause it is setting it to a random thing i would think, so that is for the not grabbed right?
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if((i-gap)%groupSize==0) { 

                m_ledBuffer.setRGB(i, 255, 0, 0);
            }
            else{m_ledBuffer.setRGB(i, 0, 0, 0);}
            }
             finish++;
             if (finish==10) {
                finish = 0;
                setRandomRelaxed();} // what is this?
    } 
    else { // is this when it doesnt have a thing?

        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if((i-gap)%groupSize==0) { 

                m_ledBuffer.setRGB(i, 0, 0, 255);
            }
            else{m_ledBuffer.setRGB(i, 0, 0, 0);}
            }
            
        }
    }


    public void runAnt(){
        for (int i = 1; i < m_ledBuffer.getLength(); i++) {
            if((i-gap)%groupSize==0) { 

                m_ledBuffer.setRGB(i, 255, 255, 0);
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

                m_ledBuffer.setRGB(i, 255, 0, 210);
            }    
            else{m_ledBuffer.setRGB(i, 255, 0, 0);}
        }
        m_led.setData(m_ledBuffer);
        gapReset();
    }

    public void setSolidColor(int r, int g, int b){
        for (var i = 0; i < m_ledBuffer.getLength(); i++){
            m_ledBuffer.setRGB(i, r, g, b);
            m_led.setData(m_ledBuffer);
        }
    }

    @Override
	public void periodic() {

        SmartDashboard.putNumber("LED mode", getSelected());

        time++; // 1 time = 20 miliseconds pretty sure
        if (time==limiter) { // updates selected light animation (20*time = miliseconds)
            update(getSelected()); 
            time=0;
        }
    }
}