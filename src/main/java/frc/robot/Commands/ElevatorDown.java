package frc.robot.Commands;
import java.lang.annotation.Target;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDown extends Command {
    
    private final ElevatorSubsystem m_ElevatorSubsystem;

    private int setPoint;

        public ElevatorDown(ElevatorSubsystem elevator, int target){
        
             m_ElevatorSubsystem = elevator;
             setPoint = target;

             addRequirements(m_ElevatorSubsystem);

        }
    
   @Override
    public void initialize (){

    }

    @Override
    public void end(boolean interrupted){

        m_ElevatorSubsystem.stop();
    }

    @Override
    public void execute(){
       
        m_ElevatorSubsystem.Backward();
    
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
