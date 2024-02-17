package frc.robot.subsystems;
//need to import the motor

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //need to initialize the motor
    private final CANSparkMax intakeMotor;
    private final DigitalInput limitSwitch;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(Constants.Intake.idleMode);
        intakeMotor.setSmartCurrentLimit(Constants.Intake.currentLimit);

        limitSwitch = new DigitalInput(Constants.Intake.limitSwitchPort);
    }
    
    public void stop() {
        intakeMotor.set(0);
    }

    public Command smartIntake() {
        return this.run(() -> intakeMotor.set(Constants.Intake.speed))
            .until(limitSwitch::get);
    } 
}
