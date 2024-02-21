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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpBarSubsystem extends SubsystemBase {
    private final CANSparkMax ampBarMotor;
    private final SparkPIDController ampBarPID;
    private final SparkAbsoluteEncoder encoder;

    private final TrapezoidProfile profile;
    private TrapezoidProfile.State start;
    private TrapezoidProfile.State goal;
    private double t;

    public AmpBarSubsystem() {
        ampBarMotor = new CANSparkMax(Constants.AmpBar.motorID, MotorType.kBrushless);
        ampBarMotor.restoreFactoryDefaults();
        ampBarMotor.setIdleMode(IdleMode.kBrake);
        ampBarMotor.setSmartCurrentLimit(Constants.AmpBar.currentLimit);

        ampBarMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        ampBarMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        ampBarMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, Constants.AmpBar.forwardSoftLimit);
        ampBarMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, Constants.AmpBar.reverseSoftLimit);

        encoder = ampBarMotor.getAbsoluteEncoder(Type.kDutyCycle);

        ampBarPID = ampBarMotor.getPIDController();
        ampBarPID.setFeedbackDevice(encoder);
        ampBarPID.setP(Constants.AmpBar.PID.P);
        ampBarPID.setI(Constants.AmpBar.PID.I);
        ampBarPID.setD(Constants.AmpBar.PID.D);
        ampBarPID.setFF(Constants.AmpBar.PID.F);

        ampBarMotor.burnFlash();


        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.AmpBar.maxVelocity, Constants.AmpBar.maxAcceleration));
    }

    public void stop() {
        ampBarMotor.set(0);
    }

    public void set(double speed) {
        ampBarMotor.set(speed);
    }

    public Command homeCommand() {
        return Commands.sequence(
            new InstantCommand(() -> {
                start = getState();
                goal = new TrapezoidProfile.State(Constants.AmpBar.homePosition, 0);
                t = 0;
                profile.calculate(t, start, goal);
            }),
            run(() -> {
                ampBarPID.setReference(profile.calculate(t, start, goal).position, CANSparkMax.ControlType.kPosition);
                t += 0.02;
            }).until(() -> profile.isFinished(t))
        );
    }

    public Command upCommand() {
        return Commands.sequence(
            new InstantCommand(() -> {
                start = getState();
                goal = new TrapezoidProfile.State(Constants.AmpBar.upPosition, 0);
                t = 0;
                profile.calculate(t, start, goal);
            }),
            run(() -> {
                ampBarPID.setReference(profile.calculate(t, start, goal).position, CANSparkMax.ControlType.kPosition);
                t += 0.02;
            }).until(() -> profile.isFinished(t))
        );
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    private TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getPosition(), getVelocity());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("amp bar position", encoder.getPosition());
        SmartDashboard.putNumber("amp bar motor applied output", ampBarMotor.getAppliedOutput());
        SmartDashboard.putNumber("amp bar motor current", ampBarMotor.getOutputCurrent());
        SmartDashboard.putNumber("amp bar setpoint", profile.calculate(t, start, goal).position);
        SmartDashboard.putNumber("amp bar target velocity", profile.calculate(t, start, goal).velocity);
        SmartDashboard.putNumber("amp bar start", start.position);
        SmartDashboard.putNumber("amp bar t", t);
    }
}
