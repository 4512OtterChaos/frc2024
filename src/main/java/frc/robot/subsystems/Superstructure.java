package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;


public class Superstructure {
    private SwerveDrive drive;
    private Intake intake;
    // private Arm arm;
    private Shooter shooter;
    private Feeder feeder;

    public Superstructure(SwerveDrive drive, Intake intake, Shooter shooter, Feeder feeder) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.feeder = feeder;
    }

    public Command shootSubwoof(){
        return sequence(
            shooter.shootSubwoofC(),
            waitSeconds(0.5),
            feeder.setVoltageInC(),
            waitSeconds(1)
        ).finallyDo(()-> {
            shooter.stop();
            feeder.setVoltage(0);
        });
    }

    public Command intake(){
        return parallel(
            intake.setVoltageInC(),
            feeder.setVoltageInC()
        ).until(()->feeder.isNoteSensed()).finallyDo(()->{
            intake.setVoltage(0);
            feeder.setVoltage(0);
        });
    }

    public Command outtake(){
        return parallel(
            intake.setVoltageOutC(),
            feeder.setVoltageOutC()
        ).finallyDo(()->{
            intake.setVoltage(0);
            feeder.setVoltage(0);
        });
    }


}
