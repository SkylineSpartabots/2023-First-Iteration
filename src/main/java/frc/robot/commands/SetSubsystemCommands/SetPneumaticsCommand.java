package frc.robot.commands.SetSubsystemCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem.PneumaticsSubsystemMode;

public class SetPneumaticsCommand extends CommandBase{
    private final PneumaticsSubsystem m_subsystem;
    private final PneumaticsSubsystemMode mode;
    private Timer m_timer;

    public SetPneumaticsCommand(PneumaticsSubsystemMode mode) {
        m_subsystem = PneumaticsSubsystem.getInstance();
        addRequirements(m_subsystem);
        this.mode = mode;
        this.m_timer = new Timer();
    }

    @Override
    public void initialize() {
      m_timer.reset();
      m_timer.start();
    }

    @Override
    public void execute() {
      m_subsystem.SetPneumaticsMode(mode);
    }

    @Override
    public boolean isFinished() {

      return m_timer.hasElapsed(5.0);
    }
    
}
