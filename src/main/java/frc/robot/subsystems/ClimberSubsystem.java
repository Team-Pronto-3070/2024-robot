package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.databind.ser.std.BooleanSerializer;

/*
 * The Climber Subsystem
 */




public class ClimberSubsystem extends SubsystemBase {
    private final WPI_TalonSRX leftTalon;
    private final WPI_TalonSRX rightTalon;

    public ClimberSubsystem() {
        leftTalon  = new WPI_TalonSRX(Constants.ClimberSubsystem.leftTalonID);
        rightTalon = new WPI_TalonSRX(Constants.ClimberSubsystem.rightTalonID);

        leftTalon .configFactoryDefault();
        rightTalon.configFactoryDefault();
        leftTalon .configAllSettings(Constants.ClimberSubsystem.leftTalonConfig);
        rightTalon.configAllSettings(Constants.ClimberSubsystem.leftTalonConfig);
        leftTalon .setNeutralMode(NeutralMode.Brake);
        rightTalon.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * 
     * @param speed in percent of *climb* speed
     */
    public void setLeftSpeed(double speed) {
        this.leftTalon.set(speed * Constants.ClimberSubsystem.maxSpeed);
    }

    /**
     * 
     * @param speed in percent of *climb* speed 
     */
    public void setRightSpeed(double speed) {
        this.rightTalon.set(speed * Constants.ClimberSubsystem.maxSpeed);
    }

    /**
     * 
     * @param speed in percent of *climb* speed
     */
    public void setBothSpeed(double speed) {
        this.setLeftSpeed(speed);
        this.setRightSpeed(speed);
    }

    /**
     * 
     * @param roll in radians. positive roll is right side higher
     * @return
     */
    public Command balancedClimbCommand(DoubleSupplier roll, BooleanSupplier enable, BooleanSupplier limitSwitch) {
        // TODO positive roll is assumed to make the right side higher than the left

        return this.run(() -> {
            double sinRoll = Math.sin(roll.getAsDouble());
            this.setLeftSpeed(
                (1.0 - sinRoll)                                     // * get the difference of height
                * Constants.ClimberSubsystem.balanceAdjustQuotient  // * how much to adjust
                * Constants.ClimberSubsystem.climbSpeed);           // * multiply get the actual motor speed
            this.setRightSpeed(
                (1.0 + sinRoll)
                * Constants.ClimberSubsystem.balanceAdjustQuotient
                * Constants.ClimberSubsystem.climbSpeed);
        }).until(() -> (!enable.getAsBoolean()) && limitSwitch.getAsBoolean());
    }

}