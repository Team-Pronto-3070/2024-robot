package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/*
 * The Climber Subsystem
 */

public class ClimberSubsystem extends SubsystemBase {

  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  public final DigitalInput leftLimitSwitch;
  public final DigitalInput rightLimitSwitch;

  public ClimberSubsystem() {
    leftMotor = new CANSparkMax(Constants.Climber.leftID, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.Climber.rightID, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setInverted(true);

    leftMotor.setIdleMode(Constants.Climber.idleMode);
    rightMotor.setIdleMode(Constants.Climber.idleMode);
    leftMotor.setSmartCurrentLimit(Constants.Climber.currentLimit);
    rightMotor.setSmartCurrentLimit(Constants.Climber.currentLimit);
    leftMotor.burnFlash();
    rightMotor.burnFlash();

    leftLimitSwitch = new DigitalInput(Constants.Climber.leftLimitSwitchPort);
    rightLimitSwitch = new DigitalInput(Constants.Climber.rightLimitSwitchPort);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  /**
   *
   * @param speed in percent of *climb* speed
   */
  public void setLeftSpeed(double speed) {
    this.leftMotor.set(speed * Constants.Climber.maxSpeed);
  }

  /**
   *
   * @param speed in percent of *climb* speed
   */
  public void setRightSpeed(double speed) {
    this.rightMotor.set(speed * Constants.Climber.maxSpeed);
  }

  /**
   *
   * @param speed in percent of *climb* speed
   */
  public void setBothSpeed(double speed) {
    this.setLeftSpeed(speed);
    this.setRightSpeed(speed);
  }

  public Command upCommand() {
    return run(() -> setBothSpeed(1));
  }

  public Command downCommand() {
    return run(() -> setBothSpeed(-1))
          //.until(leftLimitSwitch::get)
          //.until(rightLimitSwitch::get)
          .until(new Trigger(() -> leftMotor.getOutputCurrent() > 35).debounce(0.1))
          .until(new Trigger(() -> rightMotor.getOutputCurrent() > 35).debounce(0.1));
  }

  /**
   *
   * @param roll in radians. positive roll is right side higher
   * @return
   */
  public Command balancedClimbCommand(DoubleSupplier roll) {
    // TODO positive roll is assumed to make the right side higher than the left

    return this.run(() -> {
        double sinRoll = Math.sin(roll.getAsDouble());
        this.setLeftSpeed(
            (1.0 + sinRoll) * // * get the difference of height
            Constants.Climber.balanceAdjustQuotient * // * how much to adjust
            Constants.Climber.climbSpeed
          ); // * multiply get the actual motor speed
        this.setRightSpeed(
            (1.0 - sinRoll) *
            Constants.Climber.balanceAdjustQuotient *
            Constants.Climber.climbSpeed
          );
      })
      .until((new Trigger(() -> leftMotor.getOutputCurrent() > 35).debounce(0.5)))
      .until((new Trigger(() -> rightMotor.getOutputCurrent() > 35).debounce(0.5)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left climber current", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("right climber current", rightMotor.getOutputCurrent());
  }
}
