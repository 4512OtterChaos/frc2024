package frc.robot.subsystems.superShooter.arm;

import com.ctre.phoenix6.hardware.TalonFX;

public class Arm {
    private TalonFX leftMotor = new TalonFX(1);
    private TalonFX rightMotor = new TalonFX(2);

    public Arm() {

    }
}
