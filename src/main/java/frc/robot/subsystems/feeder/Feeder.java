package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCSparkMax;

public class Feeder extends SubsystemBase {
    private OCSparkMax feederMotor = new OCSparkMax(1, MotorType.kBrushless);

    public Feeder(){

    }

    public void setFeederSpeed(double speedPercent){
        MathUtil.clamp(speedPercent, -1, 1);
        feederMotor.setVoltage(speedPercent*kFeederVoltage);
    }

    public Command CSetFeedSpeed(double speedPercent){
        return run(()->setFeederSpeed(speedPercent));
    }

}
