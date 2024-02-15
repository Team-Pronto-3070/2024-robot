package frc.robot.subsystems;
//need to import the motor

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //need to initialize the motor
    private final CANSparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(Constants.Intake.idleMode);
    }
    
    public Command motorStop() {
        return this.run( ()-> intakeMotor.set(.0));
    }

    //public Command motorRun() {
        //return this.run( ()-> intakeMotor.set(Constants.IntakeSubsystem.runningSpeed));
    //}

    public Command run() {
        if (intakeMotor.getForwardLimitSwitch(Constants.Intake.limitSwitchPolarity).isPressed()) {
            //motorRun();
            return this.motorStop();
        } else {
            //motorStop();
            return this.run( ()-> intakeMotor.set(Constants.Intake.runningSpeed))
                .until(()-> intakeMotor.getForwardLimitSwitch(Constants.Intake.limitSwitchPolarity).isPressed());
        }
        
    } 
}
