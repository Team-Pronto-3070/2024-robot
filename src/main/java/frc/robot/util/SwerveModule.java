package frc.robot.util;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {

  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;
  private final AbsoluteEncoder turningAbsoluteEncoder;

  private final SparkPIDController turningPID;
  private final double chassisAngularOffset;
  private final SimpleMotorFeedforward driveFeedforward;

  private Rotation2d lastAngle;

  public SwerveModule(int driveMotorID, int turnMotorID, double encoderOffset) {
    driveMotor = new TalonFX(driveMotorID);

    var talonFXConfigurator = driveMotor.getConfigurator();

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.Inverted = Constants.Swerve.Drive.motorInvert;
    motorConfigs.NeutralMode = Constants.Swerve.Drive.neutralMode;
    talonFXConfigurator.apply(motorConfigs);

    // set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kV = Constants.Swerve.Drive.PID.F;
    slot0Configs.kP = Constants.Swerve.Drive.PID.P;
    slot0Configs.kI = Constants.Swerve.Drive.PID.I;
    slot0Configs.kD = Constants.Swerve.Drive.PID.D;


    // apply gains, 50 ms total timeout
    talonFXConfigurator.apply(slot0Configs, 0.050);

    var currentLimitsConfig = new CurrentLimitsConfigs();
    currentLimitsConfig.SupplyCurrentLimitEnable = Constants.Swerve.Drive.enableCurrentLimit;
    currentLimitsConfig.SupplyCurrentLimit = Constants.Swerve.Drive.continuousCurrentLimit;
    currentLimitsConfig.SupplyTimeThreshold = Constants.Swerve.Drive.peakCurrentDuration;
    currentLimitsConfig.SupplyCurrentThreshold = Constants.Swerve.Drive.peakCurrentLimit;
    talonFXConfigurator.apply(currentLimitsConfig);

    var OpenLoopRampsConfigs = new OpenLoopRampsConfigs();
    OpenLoopRampsConfigs.DutyCycleOpenLoopRampPeriod = Constants.Swerve.Drive.openLoopRamp;
    talonFXConfigurator.apply(OpenLoopRampsConfigs);

    var ClosedLoopRampsConfigs = new ClosedLoopRampsConfigs();
    ClosedLoopRampsConfigs.DutyCycleClosedLoopRampPeriod = Constants.Swerve.Drive.closedLoopRamp;
    talonFXConfigurator.apply(ClosedLoopRampsConfigs);

    turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
    turnMotor.restoreFactoryDefaults();
    turnMotor.setIdleMode(Constants.Swerve.Turn.idleMode);
    turnMotor.setSmartCurrentLimit(Constants.Swerve.Turn.currentLimit);

    turningAbsoluteEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    turningAbsoluteEncoder.setPositionConversionFactor(Constants.Swerve.Turn.encoderPositionFactor);
    turningAbsoluteEncoder.setVelocityConversionFactor(Constants.Swerve.Turn.encoderVelocityFactor);
    turningAbsoluteEncoder.setInverted(Constants.Swerve.Turn.encoderInvert);

    turningPID = turnMotor.getPIDController();
    turningPID.setFeedbackDevice(turningAbsoluteEncoder);
    turningPID.setPositionPIDWrappingEnabled(true);
    turningPID.setPositionPIDWrappingMinInput(0);
    turningPID.setPositionPIDWrappingMaxInput(Constants.Swerve.Turn.encoderPositionFactor);
    turningPID.setP(Constants.Swerve.Turn.PID.P);
    turningPID.setI(Constants.Swerve.Turn.PID.I);
    turningPID.setD(Constants.Swerve.Turn.PID.D);
    turningPID.setFF(Constants.Swerve.Turn.PID.F);
    turningPID.setOutputRange(Constants.Swerve.Turn.PID.minOutput, Constants.Swerve.Turn.PID.maxOutput);

    turnMotor.burnFlash();

    chassisAngularOffset = encoderOffset;
    lastAngle = new Rotation2d(chassisAngularOffset);

    driveFeedforward =
      new SimpleMotorFeedforward(
        Constants.Swerve.Drive.Feedforward.KS,
        Constants.Swerve.Drive.Feedforward.KV,
        Constants.Swerve.Drive.Feedforward.KA
      );
  }

  public void setDesiredState(SwerveModuleState2 rawDesiredState, boolean isOpenLoop) {
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
      new SwerveModuleState(
        rawDesiredState.speedMetersPerSecond,
        rawDesiredState.angle.plus(new Rotation2d(chassisAngularOffset))
      ),
      new Rotation2d(turningAbsoluteEncoder.getPosition())
    );

    if (isOpenLoop) {
      driveMotor.setControl(new DutyCycleOut(optimizedDesiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed));
    } else {
      double wheelRPM = ((optimizedDesiredState.speedMetersPerSecond * 60) / Constants.Swerve.wheelCircumference);
      double motorRPM = wheelRPM * Constants.Swerve.gearRatio;
      double motorRPS = motorRPM / 60;
      // double sensorCounts = motorRPM * (2048.0 / 600.0);
      driveMotor.setControl(new VelocityDutyCycle(motorRPS)
                .withFeedForward(driveFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond)));
    }

    // Prevent rotating module if speed is less then 1% in order to prevent
    // jittering
    Rotation2d angle = (Math.abs(optimizedDesiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                          ? lastAngle : optimizedDesiredState.angle;
    lastAngle = angle;
    turningPID.setReference(
      angle.getRadians(),
      CANSparkMax.ControlType.kPosition,
      0,
      Constants.Swerve.Turn.KV * rawDesiredState.omegaRadPerSecond
    );
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      (driveMotor.getRotorPosition().getValue() //motor rotations
      / Constants.Swerve.gearRatio) //wheel rotations
      * Constants.Swerve.wheelCircumference, // wheel meters
      new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffset)
    );
  }

  public SwerveModuleState2 getState() {
    return new SwerveModuleState2(
      (driveMotor.getRotorVelocity().getValue() //motor rot/s
      / Constants.Swerve.gearRatio) // wheel rot/s
      * Constants.Swerve.wheelCircumference, // wheel surface speed in meters per second
      new Rotation2d(turningAbsoluteEncoder.getPosition() - chassisAngularOffset),
      turningAbsoluteEncoder.getVelocity()
    );
  }
}
