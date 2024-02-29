package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {

  private CommandXboxController driver;
  private CommandXboxController operator;

  public final Trigger interruptButton;

  public final DoubleSupplier drive_x;
  public final DoubleSupplier drive_y;
  public final DoubleSupplier drive_rot;

  public final Trigger driveFast;
  public final DoubleSupplier driveBoost;

  public final DoubleSupplier processed_drive_x;
  public final DoubleSupplier processed_drive_y;
  public final DoubleSupplier processed_drive_rot;

  public final Trigger gyroResetButton;

  public final Trigger speakerPrepButton;
  public final Trigger ampPrepButton;
  public final Trigger fireButton;

  public final Trigger smartIntakeButton;

  public final Trigger climberUpButton;
  public final Trigger climberDownButton;



  public final Trigger ampHomeButton;
  public final Trigger ampUpButton;
  public final Trigger ampManualUpButton;
  public final Trigger ampManualDownButton;
  public final Trigger manualIntakeButton;
  public final Trigger manualOuttakeButton;
  public final Trigger climberManualOverrideButton;
  public final DoubleSupplier climberLeftSpeed;
  public final DoubleSupplier climberRightSpeed;


  public OI() {
    driver = new CommandXboxController(Constants.OI.driverPort);
    operator = new CommandXboxController(Constants.OI.operatorPort);

    interruptButton = driver.start().or(operator.start());

    drive_x = () -> -driver.getLeftY();
    drive_y = () -> -driver.getLeftX();
    drive_rot = () -> -driver.getRightX();

    driveFast = driver.leftTrigger(Constants.OI.triggerDeadband);
    driveBoost = () -> driver.getLeftTriggerAxis();

    //processed_drive_x = () -> Math.pow( MathUtil.applyDeadband(drive_x.getAsDouble(), Constants.OI.deadband), 3)
                            //* Constants.Swerve.maxSpeed * (driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1);
    //processed_drive_y = () -> Math.pow( MathUtil.applyDeadband(drive_y.getAsDouble(), Constants.OI.deadband), 3)
                            //* Constants.Swerve.maxSpeed * (driveSlow.getAsBoolean() ? Constants.OI.slowSpeed : 1);
    //processed_drive_rot = () -> Math.pow( MathUtil.applyDeadband( drive_rot.getAsDouble(), Constants.OI.deadband), 3)
                            //* Constants.Swerve.maxAngularSpeed * (driveSlow.getAsBoolean() ? 0.25 : 1);
    processed_drive_x = () -> Math.pow( MathUtil.applyDeadband(drive_x.getAsDouble(), Constants.OI.deadband), 3)
                            * Constants.Swerve.maxSpeed * (driveFast.getAsBoolean() ? 
                                      Constants.OI.slowSpeed + MathUtil.applyDeadband(driveBoost.getAsDouble(), Constants.OI.triggerDeadband, 1 - Constants.OI.slowSpeed)
                                      : Constants.OI.slowSpeed);
    processed_drive_y = () -> Math.pow( MathUtil.applyDeadband(drive_y.getAsDouble(), Constants.OI.deadband), 3)
                            * Constants.Swerve.maxSpeed * (driveFast.getAsBoolean() ? 
                                      Constants.OI.slowSpeed + MathUtil.applyDeadband(driveBoost.getAsDouble(), Constants.OI.triggerDeadband, 1 - Constants.OI.slowSpeed)
                                      : Constants.OI.slowSpeed);
    processed_drive_rot = () -> Math.pow( MathUtil.applyDeadband(drive_rot.getAsDouble(), Constants.OI.deadband), 3)
                            * Constants.Swerve.maxAngularSpeed * (driveFast.getAsBoolean() ? 
                                      Constants.OI.slowSpeed + MathUtil.applyDeadband(driveBoost.getAsDouble(), Constants.OI.triggerDeadband, 1 - Constants.OI.slowSpeed)
                                      : Constants.OI.slowSpeed) * 0.75;

    // driveSlow = driver.rightTrigger();
    gyroResetButton = driver.x();

    speakerPrepButton = driver.rightBumper().or(operator.rightBumper());
    ampPrepButton = driver.rightTrigger().or(operator.rightTrigger());
    fireButton = driver.a();

    smartIntakeButton = driver.leftBumper();

    climberUpButton = driver.y();
    climberDownButton = driver.povDown();

    ampHomeButton = operator.x();
    ampUpButton = operator.y();
    ampManualUpButton = operator.povUp();
    ampManualDownButton = operator.povDown();
    manualIntakeButton = operator.a();
    manualOuttakeButton = operator.b();
    climberManualOverrideButton = operator.leftBumper();
    climberLeftSpeed = () -> -operator.getLeftY();
    climberRightSpeed = () -> -operator.getRightY();
  }
}