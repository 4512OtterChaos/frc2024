package frc.robot.subsystems.superShooter.feeder;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.OCSparkMax;

public class Feeder {
    private OCSparkMax feederMotor = new OCSparkMax(1, MotorType.kBrushless);

    public Feeder(){

    }
}
