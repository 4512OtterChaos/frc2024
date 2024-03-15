package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.OCSparkMax;

public class Feeder extends SubsystemBase {
    private OCSparkMax feederMotor = new OCSparkMax(kMotorID, MotorType.kBrushless);
    private DigitalInput sensor = new DigitalInput(kSensorID);

    private double lastFreeTime;

    public Feeder(){

    }

    @Override
    public void periodic() {
        if (getFeederCurrent() <= kStallCurrent){
            lastFreeTime = Timer.getFPGATimestamp();
        }
    }

    public boolean isNoteSensed(){
        return !sensor.get();
    }

    public double getFeederCurrent(){
        return feederMotor.getOutputCurrent();
    }

    public boolean isStalled(){
        return Timer.getFPGATimestamp() >= (lastFreeTime + kStallTime);
    }

    public void setVoltage(double voltage){
        MathUtil.clamp(-voltage, -12, 12);
        feederMotor.setVoltage(voltage);
    }

    //---------- Command factories

    /** Sets the feeder voltage and ends immediately. */
    public Command setVoltageC(double voltage){
        return runOnce(()->setVoltage(voltage));
    }
    
    /** Sets the feeder voltage for intaking and ends immediately. */
    public Command setVoltageInC(){
        return runOnce(()->setVoltage(3));
    }

    /** Sets the feeder voltage for outtaking and ends immediately. */
    public Command setVoltageOutC(){
        return runOnce(()->setVoltage(-4));
    }

}
