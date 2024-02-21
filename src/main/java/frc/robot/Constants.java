package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static final class Swerve {
    public static final IdleMode idleMode = IdleMode.kBrake;

    public static final double wheelBase = Units.inchesToMeters(18.0 - 3.5); // distance between front and back // wheels
    public static final double trackWidth = Units.inchesToMeters(18.0 - 3.5); // distance between left and right // wheels
    public static final double wheelCircumference = Units.inchesToMeters(3) * Math.PI;

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a robot that drives faster).
    private static final int drivingMotorPinionTeeth = 14;
    public static final double gearRatio =
      (45.0 * 22) / (drivingMotorPinionTeeth * 15);

    public static final double maxSpeed = Units.feetToMeters(15.87); // meters per second
    public static final double maxAcceleration = 1.19 * 9.81; // traction limited: COF*g (TODO: this COF is for blue nitrile on carpet)
    // public static final double maxAngularSpeed = 10.0 * maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);
    public static final double maxAngularSpeed = 1.0 * maxSpeed / Math.hypot(wheelBase / 2.0, trackWidth / 2.0);

    // offsets are in radians
    public static final class FrontLeft {
      public static final int driveID = 2;
      public static final int turnID = 6;
      public static final double offset = -Math.PI / 2;
    }

    public static final class FrontRight {
      // public static final int driveID = 3;
      // public static final int turnID = 7;
      public static final int driveID = 1;
      public static final int turnID = 5;
      public static final double offset = 0.0;
    }

    public static final class RearLeft {
      // public static final int driveID = 2;
      // public static final int turnID = 6;
      public static final int driveID = 4;
      public static final int turnID = 8;
      public static final double offset = Math.PI;
    }

    public static final class RearRight {
      // public static final int driveID = 1;
      // public static final int turnID = 5;
      public static final int driveID = 3;
      public static final int turnID = 7;
      public static final double offset = Math.PI / 2;
    }

    public static final class Drive {
      public static final class PID {
        public static final double P = 0.02;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
      }

      public static final class Feedforward {
        public static final double KS = 0.0;
        // public static final double KV = 12 / maxSpeed;
        public static final double KV = 0.0;
        // public static final double KA = 12 / maxAcceleration;
        public static final double KA = 0.0;
      }

      public static final int continuousCurrentLimit = 35;
      public static final int peakCurrentLimit = 60;
      public static final double peakCurrentDuration = 0.1;
      public static final boolean enableCurrentLimit = true;

      public static final double openLoopRamp = 0.25;
      public static final double closedLoopRamp = 0.0;

      public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive; // TODO: Is clockwise or counter clockwise correct?
      public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    }

    public static final class Turn {
      public static final double encoderPositionFactor = 2 * Math.PI; // radians
      public static final double encoderVelocityFactor = 2 * Math.PI / 60.0; // radians per second
      public static final boolean encoderInvert = true;

      public static final double maxModuleAngularSpeed = Units.rotationsPerMinuteToRadiansPerSecond( // radians per second
        11000.0 * // NEO550 free speed (rpm)
        203.0 /
        9424.0
      ); // gear ratio
      public static final double KV = 12.0 / maxModuleAngularSpeed; // volts * seconds / radians

      public static final class PID {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
        public static final double minOutput = -1;
        public static final double maxOutput = 1;
      }

      public static final IdleMode idleMode = IdleMode.kBrake;
      public static final int currentLimit = 20;
    }
  }

  public static final class OI {
    public static final int driverPort = 0;
    public static final int operatorPort = 1;
    public static final double deadband = 0.20;
    public static final double triggerDeadband = 0.5;

    public static final double slowSpeed = 0.5;
  }

  public static final class Shooter {
    public static final int leftMotorID = 9;
    public static final int rightMotorID = 10;

    public static final class Motor {
      public static final boolean invert = true;
      public static final IdleMode idleMode = IdleMode.kCoast;
      public static final int encoderMeasurementPeriod = 16;
      public static final int encoderMovingAverageDepth = 2;
      public static final int currentLimit = 30;
      public static final double closedLoopRampTime = 0.5; //seconds

      public static final double ampSpeed = 1000.0; // Motor speed in RPM
      public static final double speakerSpeed = 3000.0; // Motor speed in RPM
      public static final double rightMod = 1.0;
      public static final double RPMtolerance = 200;

      public static final class PID {
        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;
        public static final double F = 0.0;
      }
    }
  }

  public static final class Intake {
      public static final double speed = 0.2;
      public static final int limitSwitchPort = 0;
      public static final int motorID = 13;
      public static final IdleMode idleMode = IdleMode.kBrake;
      public static final int currentLimit = 20;
  }

  public static final class Climber {
      public static final int leftID = 11;
      public static final int rightID = 12;
      public static final int leftLimitSwitchPort = 1;
      public static final int rightLimitSwitchPort = 2;
      public static final IdleMode idleMode = IdleMode.kBrake;
      public static final int currentLimit = 40;

      public static final double climbSpeed = 0.2; // in percent output
      public static final double maxSpeed = 1; // in percent output

      public static final double balanceAdjustQuotient = 0.5; // TODO

  }
  public static final class Autos {
    public static final PIDConstants translationPID = new PIDConstants(1, 0, 0);
    public static final PIDConstants rotationPID = new PIDConstants(1, 0, 0);

    // TODO fill out
    public static final double maxSpeed = 0.0; // in meters per second
    public static final double driveBaseRadius = 0.0; // in meters
  }
}
