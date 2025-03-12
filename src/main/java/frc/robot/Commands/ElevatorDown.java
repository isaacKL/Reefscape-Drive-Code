package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ElevatorDown extends Command {
    
    private final ClimberSubsystem m_ClimberSubsystem;

        public ElevatorDown(ClimberSubsystem climber){
        
             m_ClimberSubsystem = climber;

             addRequirements(m_ClimberSubsystem);

        }
    
   @Override
    public void initialize (){

    }

    @Override
    public void end(boolean interrupted){

        m_ClimberSubsystem.stop();
    }

    @Override
    public void execute(){
       
        m_ClimberSubsystem.Backward();
    }

}
