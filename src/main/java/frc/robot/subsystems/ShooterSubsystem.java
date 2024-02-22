package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.ShooterModule;

public class ShooterSubsystem extends SubsystemBase {

  public static enum Target {SPEAKER, AMP, NONE}
  private Target target;

  private double ampRPM = Constants.Shooter.Motor.ampSpeed;
  private double speakerRPM = Constants.Shooter.Motor.speakerSpeed;
  private double rightMod = Constants.Shooter.Motor.rightMod;

  private final ShooterModule motorLeft;
  private final ShooterModule motorRight;

  public ShooterSubsystem() {
    motorLeft = new ShooterModule(Constants.Shooter.leftMotorID);
    motorRight = new ShooterModule(Constants.Shooter.rightMotorID);

    SmartDashboard.putNumber("Speaker Note RPM", speakerRPM);
    SmartDashboard.putNumber("Amp Note RPM", ampRPM);
    SmartDashboard.putNumber("Right Flywheel Multiplier", rightMod);
  }

  public boolean atTarget() {
    if (target == Target.SPEAKER) {
      return MathUtil.isNear(speakerRPM, motorLeft.getRPM(), Constants.Shooter.Motor.RPMtolerance)
          && MathUtil.isNear(speakerRPM * -rightMod, motorRight.getRPM(), Constants.Shooter.Motor.RPMtolerance);
    } else if (target == Target.AMP) {
      return MathUtil.isNear(ampRPM, motorLeft.getRPM(), Constants.Shooter.Motor.RPMtolerance)
          && MathUtil.isNear(ampRPM * -rightMod, motorRight.getRPM(), Constants.Shooter.Motor.RPMtolerance);
    }
    return false;
  }

  public void stop() {
    motorLeft.stop();
    motorRight.stop();
  }

  public Command prepSpeakerCommand() {
    return Commands.parallel(
      new InstantCommand(() -> target = Target.SPEAKER),
      run(() -> {
        speakerRPM = SmartDashboard.getNumber("Speaker Note RPM", speakerRPM);
        rightMod = SmartDashboard.getNumber("Right Flywheel Multiplier", rightMod);
        motorLeft.setRPM(speakerRPM);
        motorRight.setRPM(speakerRPM * -rightMod);
      })
    );
  }

  public Command prepAmpCommand(AmpBarSubsystem ampBar) {
    return Commands.parallel(
      new InstantCommand(() -> target = Target.AMP),
      ampBar.upCommand(),
      run(() -> {
        ampRPM = SmartDashboard.getNumber("Amp Note RPM", ampRPM);
        rightMod = SmartDashboard.getNumber("Right Flywheel Multiplier", rightMod);
        motorLeft.setRPM(ampRPM);
        motorRight.setRPM(ampRPM * -rightMod);
      })
    );
  }

  public Command fireCommand(IntakeSubsystem intake, AmpBarSubsystem ampBar) {
    return Commands.sequence(
      Commands.waitUntil(new Trigger(this::atTarget).debounce(0.5)
                    .and(new Trigger(() -> (target != Target.AMP) || 
                        MathUtil.isNear(Constants.AmpBar.upPosition, ampBar.getPosition(), Constants.AmpBar.tolerance))
                              .debounce(0.5))),
      Commands.print("fire command at target"),
      intake.run(() -> intake.set(0.2)).withTimeout(2),
      intake.runOnce(intake::stop),
      this.runOnce(this::stop),
      Commands.either(ampBar.homeCommand(), Commands.none(), () -> target == Target.AMP),
      new InstantCommand(() -> target = Target.NONE)
    );

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left shooter RPM", motorLeft.getRPM());
    SmartDashboard.putNumber("right shooter RPM", motorRight.getRPM());
    SmartDashboard.putNumber("left shooter applied output", motorLeft.getAppliedOutput());
    SmartDashboard.putNumber("left shooter current", motorLeft.getCurrent());
    SmartDashboard.putNumber("right shooter applied output", motorRight.getAppliedOutput());
    SmartDashboard.putNumber("right shooter current", motorRight.getCurrent());
    SmartDashboard.putString("shooter target", "" + target);
    SmartDashboard.putBoolean("shooter at target", atTarget());
  }
}
