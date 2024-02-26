package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCSparkMax;

public class Shooter extends SubsystemBase {
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

    public Command CShootSubwoof(){
        return run(()->shootSubwoof());
    }

    public Command CShootAmp(){
        return run(()->shootAmp());
    }

    public Command CShootTable(double distance){
        return run(()->shootTable(distance));
    }

}
