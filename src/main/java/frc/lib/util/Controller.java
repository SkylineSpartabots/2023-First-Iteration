package frc.lib.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controller {
    private XboxController xbox;
    private static final byte DPAD_U_PORT = -1;
    private static final byte DPAD_D_PORT = -3;
    private static final byte DPAD_L_PORT = -4;
    private static final byte DPAD_R_PORT = -2;
    private Button a, b, x, y, rb, lb, lstick, rstick, back, start;
    private POVButton up, down, left, right;
    public Controller(XboxController xbox){
        this.xbox = xbox;
        a = new Button(xbox::getAButton);
        b = new Button(xbox::getBButton);
        x = new Button(xbox::getXButton);
        y = new Button(xbox::getYButton);
        lb = new Button(xbox::getLeftBumper);
        rb = new Button(xbox::getRightBumper);
        lstick = new Button(xbox::getLeftStickButton);
        rstick = new Button(xbox::getRightStickButton);
        back = new Button(xbox::getBackButton);
        start = new Button(xbox::getStartButton);
    }
    
    public boolean getDpadUp(){ return xbox.getPOV() == 0; }
    public boolean getDpadUpRight(){ return xbox.getPOV() > 15 && xbox.getPOV() < 75; }
    public boolean getDpadRight(){ return xbox.getPOV() == 90;}
    public boolean getDpadDown(){ return xbox.getPOV() == 180;}
    public boolean getDpadLeft(){ return xbox.getPOV() == 270;}

    public double getRightTriggerAxis(){ return xbox.getRightTriggerAxis();}
    public double getLeftTriggerAxis(){ return xbox.getLeftTriggerAxis();}

    public Button getAButton(){ return a;}
    public Button getBButton(){ return b;}
    public Button getXButton(){ return x;}
    public Button getYButton(){ return y;}
    public Button getLeftBumper(){ return lb;}
    public Button getRightBumper(){ return rb;}
    public Button getLeftStickButton(){ return lstick;}
    public Button getRightStickButton(){ return rstick;}
    public Button getBackButton(){ return back;}
    public Button getStartButton(){ return start;}

    public double getLeftX(){ return xbox.getLeftX();}
    public double getLeftY(){ return xbox.getLeftY();}
    public double getRightX(){ return xbox.getRightX();}
    public double getRightY(){ return xbox.getRightY();}
}
