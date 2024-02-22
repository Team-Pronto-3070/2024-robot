package frc.robot.subsystems;
//need to import the motor

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        intakeMotor.burnFlash();

        limitSwitch = new DigitalInput(Constants.Intake.limitSwitchPort);
    }
    
    public void stop() {
        intakeMotor.set(0);
    }

    public void set(double speed) {
        intakeMotor.set(speed);
    }

    public Command smartIntakeCommand() {
        return this.run(() -> intakeMotor.set(Constants.Intake.speed))
            ;//.until(limitSwitch::get);
    } 

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake limit switch", limitSwitch.get());
        SmartDashboard.putNumber("intake current", intakeMotor.getOutputCurrent());
    }
}