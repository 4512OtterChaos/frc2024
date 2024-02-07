package frc.robot.subsystems.superShooter.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import static frc.robot.subsystems.superShooter.shooter.ShooterConstants.*;

import frc.robot.util.OCSparkMax;

public class Shooter {
    private OCSparkMax leftMotor = new OCSparkMax(1, MotorType.kBrushless);
    private OCSparkMax rightMotor = new OCSparkMax(1, MotorType.kBrushless);

    public Shooter() {
        rightMotor.setInverted(true);

        leftMotor.enableVoltageCompensation(kShooterVoltage);
        rightMotor.enableVoltageCompensation(kShooterVoltage);
    }

    public void shootSubwoof(){
        leftMotor.set(0.8);
        rightMotor.set(0.75);
    }

    public void shootAmp(){
        leftMotor.set(0.3);
        rightMotor.set(0.3);
    }

    public void shootTable(double distance){
    // to be completed once we have the table
    }

}
