package frc.robot.util;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class ShooterModule {

  private final CANSparkMax shooterMotor;
  private final SparkPIDController shooterPID;

  public ShooterModule(int motorPort) {
    shooterMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setIdleMode(Constants.Shooter.Motor.idleMode);
    shooterMotor.setInverted(Constants.Shooter.Motor.invert);
    shooterMotor.setSmartCurrentLimit(Constants.Shooter.Motor.currentLimit);
    shooterMotor.setClosedLoopRampRate(Constants.Shooter.Motor.closedLoopRampTime);

    shooterMotor.getEncoder().setMeasurementPeriod(Constants.Shooter.Motor.encoderMeasurementPeriod);
    shooterMotor.getEncoder().setAverageDepth(Constants.Shooter.Motor.encoderMovingAverageDepth);

    shooterPID = shooterMotor.getPIDController();
    shooterPID.setFeedbackDevice(shooterMotor.getEncoder());
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

  public void stop() {
    shooterMotor.set(0);
  }
}