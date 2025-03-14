package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonomousLeftSide extends Command{
    
     private final ClimberSubsystem m_ClimberSubsystem;

        public ElevatorUp(ClimberSubsystem climber, int target){
            
             setPoint = target;
             m_ClimberSubsystem = climber;

             addRequirements(m_ClimberSubsystem);

        }
    
   @Override
    public void initialize (){

    }

    @Override
    public void execute(){
       
        m_ClimberSubsystem.forward();
    }

    @Override
    public void end(boolean interrupted){

        m_ClimberSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    

}
