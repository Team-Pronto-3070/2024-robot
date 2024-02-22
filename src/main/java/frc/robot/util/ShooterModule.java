package frc.robot.util;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class ShooterModule {

  private final CANSparkMax shooterMotor;
  private final RelativeEncoder shooterEncoder;
  private final SparkPIDController shooterPID;

  public ShooterModule(int motorPort) {
    shooterMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(Constants.Shooter.Motor.idleMode);
    shooterMotor.setInverted(Constants.Shooter.Motor.invert);
    shooterMotor.setSmartCurrentLimit(Constants.Shooter.Motor.currentLimit);
    shooterMotor.setClosedLoopRampRate(Constants.Shooter.Motor.closedLoopRampTime);

    shooterEncoder = shooterMotor.getEncoder();
    shooterEncoder.setMeasurementPeriod(Constants.Shooter.Motor.encoderMeasurementPeriod);
    shooterEncoder.setAverageDepth(Constants.Shooter.Motor.encoderMovingAverageDepth);

    shooterPID = shooterMotor.getPIDController();
    shooterPID.setFeedbackDevice(shooterEncoder);
    shooterPID.setP(Constants.Shooter.Motor.PID.P);
    shooterPID.setI(Constants.Shooter.Motor.PID.I);
    shooterPID.setD(Constants.Shooter.Motor.PID.D);
    shooterPID.setFF(Constants.Shooter.Motor.PID.F);

    shooterMotor.burnFlash();
  }

  public void setRPM(double speed) {
    shooterPID.setReference(
      speed,
      CANSparkMax.ControlType.kVelocity,
      0
    );
  }

  public double getRPM() {
    return shooterEncoder.getVelocity();
  }

  public double getAppliedOutput() {
    return shooterMotor.getAppliedOutput();
  }

  public double getCurrent() {
    return shooterMotor.getOutputCurrent();
  }

  public void stop() {
    shooterMotor.set(0);
  }
}