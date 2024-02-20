package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpBarSubsystem extends SubsystemBase {
    private final CANSparkMax ampBarMotor;
    private final SparkPIDController ampBarPID;
    private final TrapezoidProfile profile;
    private final SparkAbsoluteEncoder encoder;

    public AmpBarSubsystem() {
        ampBarMotor = new CANSparkMax(Constants.AmpBar.motorID, MotorType.kBrushless);
        ampBarMotor.restoreFactoryDefaults();
        ampBarMotor.setIdleMode(IdleMode.kBrake);
        ampBarMotor.setSmartCurrentLimit(Constants.AmpBar.currentLimit);

        encoder = ampBarMotor.getAbsoluteEncoder(Type.kDutyCycle);

        ampBarPID = ampBarMotor.getPIDController();
        ampBarPID.setFeedbackDevice(encoder);
        ampBarPID.setP(Constants.AmpBar.PID.P);
        ampBarPID.setI(Constants.AmpBar.PID.I);
        ampBarPID.setD(Constants.AmpBar.PID.D);
        ampBarPID.setFF(Constants.AmpBar.PID.F);

        ampBarMotor.burnFlash();


        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
    }

    public void stop() {
        ampBarMotor.set(0);
    }

    public void set(double speed) {
        ampBarMotor.set(speed);
    }

    public Command homeCommand() {
        return run(() -> ampBarPID.setReference(profile.calculate(0.02, getState(),
                    new TrapezoidProfile.State(Constants.AmpBar.homePosition, 0)).position, 
                    CANSparkMax.ControlType.kPosition))
                .until(() -> profile.isFinished(0.02));
    }

    public Command upCommand() {
        return run(() -> ampBarPID.setReference(profile.calculate(0.02, getState(),
                    new TrapezoidProfile.State(Constants.AmpBar.upPosition, 0)).position, 
                    CANSparkMax.ControlType.kPosition))
                .until(() -> profile.isFinished(0.02));
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getPosition();
    }

    private TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getPosition(), getVelocity());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("amp bar position", encoder.getPosition());
        SmartDashboard.putNumber("amp bar motor applied output", ampBarMotor.getAppliedOutput());
    }
}
