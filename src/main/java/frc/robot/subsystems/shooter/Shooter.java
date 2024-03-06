package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCSparkMax;

public class Shooter extends SubsystemBase {
    private OCSparkMax leftMotor = new OCSparkMax(kLeftMotorID, MotorType.kBrushless);
    private OCSparkMax rightMotor = new OCSparkMax(kRightMotorID, MotorType.kBrushless);

    public Shooter() {
        leftMotor.enableVoltageCompensation(12);
        rightMotor.enableVoltageCompensation(12);
    }

    public void setLeftVoltage(double voltage){
        leftMotor.setVoltage(voltage);
    }
    public void setRightVoltage(double voltage){
        rightMotor.setVoltage(voltage);
    }

    public void shootSubwoof(){
        setLeftVoltage(9);
        setRightVoltage(8);
    }

    public void shootAmp(){
        setLeftVoltage(2);
        setRightVoltage(2);
    }

    public void stop(){ 
        setLeftVoltage(0);
        setRightVoltage(0);
    }

    public void shootTable(double distance){
    // to be completed once we have the table
    }

    public Command shootSubwoofC(){
        return runOnce(()->shootSubwoof());
    }

    public Command CShootAmp(){
        return run(()->shootAmp());
    }

    public Command CShootTable(double distance){
        return run(()->shootTable(distance));
    }

    public Command stopC() {
        return runOnce(()-> {
            stop();
        });
    }
}
