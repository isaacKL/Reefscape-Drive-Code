package frc.robot.subsystems;



import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link edu.wpi.first.wpilibj.DoubleSolenoid}. */
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final SparkMax pivotMotor;
    private final AbsoluteEncoder pivotEncoder; 
    private final AbsoluteEncoder shooterEncoder; 

    public ShooterSubsystem(int pivotCANId, int shooterCANId){
        //Motors
        pivotMotor = new SparkMax(pivotCANId, MotorType.kBrushless);
        shooterMotor = new SparkMax(shooterCANId, MotorType.kBrushless);
        //Encoders
        pivotEncoder = pivotMotor.getAbsoluteEncoder();
        shooterEncoder = shooterMotor.getAbsoluteEncoder();
    
    }

    /**  */
    public Command outwardCommand() {
        // implicitly require `this`
        return this.runOnce(() -> shooterMotor.set(kShooterForward));
    }

    /** Releases the hatch. */
    public Command inwardCommand() {
        // implicitly require `this`
        return this.runOnce(() -> shooterMotor.set(kShooterReverse));
    }
    
    public Command shooterStop(){   
        return this.runOnce(() -> shooterMotor.set(0));
    }

    public Command pivotStop(){   
        return this.runOnce(() -> pivotMotor.set(0));
    }

    public Command pivotUp(){   
        return this.runOnce(() -> pivotMotor.set(kPivotForward));
    }

    public Command pivotDown(){
        return this.runOnce(() -> pivotMotor.set(kPivotReverse));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // Publish the solenoid state to telemetry.
        builder.addBooleanProperty("extended", () -> m_hatchSolenoid.get() == kForward, null);
    }
}