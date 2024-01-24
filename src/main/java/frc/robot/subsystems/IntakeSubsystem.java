package frc.robot.subsystems;
//need to import the motor

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //need to initialize the motor
    private final WPI_TalonSRX intakeMotor; 

    public IntakeSubsystem() {
        intakeMotor = new WPI_TalonSRX(Constants.IntakeSubsystem.intakeMotorID);
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }
    
    public Command motorStop() {
        return this.run( ()-> intakeMotor.set(.0));
    }

    //public Command motorRun() {
        //return this.run( ()-> intakeMotor.set(Constants.IntakeSubsystem.runningSpeed));
    //}

    public Command run() {
        if (intakeMotor.isFwdLimitSwitchClosed() == 1) {
            //motorRun();
            return this.motorStop();
        } else {
            //motorStop();
            return this.run( ()-> intakeMotor.set(Constants.IntakeSubsystem.runningSpeed))
                .until(()-> intakeMotor.isFwdLimitSwitchClosed() == 1);
        }
        
    } 
}
