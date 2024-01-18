package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
     * @param speed in percent output
     */
    public void setLeftSpeed(double speed) {
        this.leftTalon.set(speed);
    }

    /**
     * 
     * @param speed in percent output
     */
    public void setRightSpeed(double speed) {
        this.rightTalon.set(speed);
    }

    /**
     * 
     * @param speed in percent output
     */
    public void setBothSpeed(double speed) {
        this.setLeftSpeed(speed);
        this.setRightSpeed(speed);
    }

}