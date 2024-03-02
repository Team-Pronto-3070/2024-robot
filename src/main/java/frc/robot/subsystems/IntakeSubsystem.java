package frc.robot.subsystems;
//need to import the motor

import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.Set;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    //need to initialize the motor
    private final CANSparkMax intakeMotor;
    private final DigitalInput leftBeamBreak;
    private final DigitalInput rightBeamBreak;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.Intake.motorID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setIdleMode(Constants.Intake.idleMode);
        intakeMotor.setSmartCurrentLimit(Constants.Intake.currentLimit);
        intakeMotor.burnFlash();

        leftBeamBreak = new DigitalInput(Constants.Intake.leftBeamBreakPort);
        rightBeamBreak = new DigitalInput(Constants.Intake.rightBeamBreakPort);
    }
    
    public void stop() {
        intakeMotor.set(0);
    }

    public void set(double speed) {
        intakeMotor.set(speed);
    }

    public boolean hasNote() {
        return !leftBeamBreak.get() || !rightBeamBreak.get();
    }

    public Command smartIntakeCommand() {
        return this.run(() -> intakeMotor.set(hasNote() ? 0 : Constants.Intake.speed))
            .until(this::hasNote);
    } 

    public Command smartIntakeCommand2() {
        return Commands.sequence(
            this.run(() -> intakeMotor.set(1)).until(new Trigger(() -> hasNote())),
            new ScheduleCommand(Commands.sequence(
                Commands.defer(() -> this.run(() -> intakeMotor.set(-0.15)).until(new Trigger(() -> hasNote()).debounce(0.08)), Set.of(this)),
                Commands.defer(() -> this.run(() -> intakeMotor.set(-0.2)).until(new Trigger(() -> !hasNote()).debounce(0.04)), Set.of(this)),
                Commands.defer(() -> this.run(() -> intakeMotor.set(0.1)).until(new Trigger(() -> hasNote()).debounce(0.04)), Set.of(this))
            ))
        );
    } 

    public Command smartIntakeCommand3() {
        return Commands.sequence(
            this.run(() -> intakeMotor.set(Constants.Intake.speed)).until(new Trigger(() -> hasNote()).debounce(0.02)),
            backOffCommand()
            //this.run(() -> intakeMotor.set(-0.2)).withTimeout(0.2)
        );
    } 

    public Command backOffCommand() {
        return new ScheduleCommand(this.run(() -> intakeMotor.set(-0.2)).withTimeout(0.2));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("intake left beam break", leftBeamBreak.get());
        SmartDashboard.putBoolean("intake right beam break", rightBeamBreak.get());
        SmartDashboard.putBoolean("intake has note", hasNote());
        SmartDashboard.putNumber("intake current", intakeMotor.getOutputCurrent());
        SmartDashboard.putNumber("intake applied output", intakeMotor.getAppliedOutput());
    }
}