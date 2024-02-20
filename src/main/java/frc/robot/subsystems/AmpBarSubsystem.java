package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AmpBarSubsystem extends SubsystemBase {
    private final CANSparkMax ampBarMotor;
    private final SparkPIDController ampBarPID;
    private final TrapezoidProfile profile;
    private final RelativeEncoder encoder;

    public AmpBarSubsystem() {
        ampBarMotor = new CANSparkMax(Constants.AmpBar.motorID, MotorType.kBrushless);
        ampBarMotor.restoreFactoryDefaults();
        ampBarMotor.setIdleMode(IdleMode.kBrake);
        ampBarMotor.setSmartCurrentLimit(Constants.AmpBar.currentLimit);

        ampBarPID = ampBarMotor.getPIDController();
        ampBarPID.setFeedbackDevice(ampBarMotor.getEncoder());
        ampBarPID.setP(Constants.AmpBar.PID.P);
        ampBarPID.setI(Constants.AmpBar.PID.I);
        ampBarPID.setD(Constants.AmpBar.PID.D);
        ampBarPID.setFF(Constants.AmpBar.PID.F);

        encoder = ampBarMotor.getEncoder();

        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
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

    public boolean atTarget() {
        return MathUtil.isNear(Constants.AmpBar.upPosition, encoder.getPosition(), Constants.AmpBar.tolerance);
    }

    private TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(
            encoder.getPosition(), encoder.getVelocity()
        );
    }
}
