package frc.robot.subsystems.superShooter.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.OCSparkMax;

public class Shooter {
    private OCSparkMax leftMotor = new OCSparkMax(1, MotorType.kBrushless);
    private OCSparkMax rightMotor = new OCSparkMax(1, MotorType.kBrushless);

    public Shooter() {

    }
}
