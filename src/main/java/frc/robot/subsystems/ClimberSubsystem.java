package frc.robot.subsystems;

//package edu.wpi.first.wpilibj.examples.hatchbotinlined.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** A hatch mechanism actuated by a single {@link edu.wpi.first.wpilibj.DoubleSolenoid}. */
public class ClimberSubsystem extends SubsystemBase {
  
  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;

  //private final ProfiledPIDController m_controller =
      //new ProfiledPIDController(
          //DriveConstants.kTurnP,
          //0,
          //0,
         // new TrapezoidProfile.Constraints(
             // DriveConstants.kMaxTurnRateDegPerS,
             // DriveConstants.kMaxTurnAccelerationDegPerSSquared));
  //private final SimpleMotorFeedforward m_feedforward =
      //new SimpleMotorFeedforward(
         // DriveConstants.ksVolts,
          //DriveConstants.kvVoltSecondsPerDegree,
          //DriveConstants.kaVoltSecondsSquaredPerDegree);

public ClimberSubsystem(int leftCanId){

    climberMotor = new SparkMax(leftCanId, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();


}

public void forward(){
climberMotor.set(.1);

}

public void Backward(){
  climberMotor.set(-.1);
}

public void stop(){
  climberMotor.set(0);
}

public double getDistance(){
  return climberEncoder.getVelocity();

}
  

 
  }
