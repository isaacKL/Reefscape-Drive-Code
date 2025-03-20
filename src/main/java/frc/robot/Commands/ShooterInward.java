package frc.robot.Commands;
import java.lang.annotation.Target;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterInward extends Command {
    
     private int setPoint;


    private final ShooterSubsystem m_ShooterSubsystem;

    public ShooterInward(ShooterSubsystem shooterIn, int target){
        
            setPoint = target;
            m_ShooterSubsystem = shooterIn;

            addRequirements(m_ShooterSubsystem);

    }
    
   @Override
    public void initialize (){

    }

    @Override
    public void execute(){
        double speed = m_ShooterSubsystem.shooter_cntlr.calculate(m_ShooterSubsystem.getShooterPosition(), setPoint);
        m_ShooterSubsystem.shoot(speed);
    }

    @Override
    public void end(boolean interrupted){
        m_ShooterSubsystem.shooterStop();
    }

    @Override
    public boolean isFinished(){
        return m_ShooterSubsystem.shooter_cntlr.atGoal();
    }
}
