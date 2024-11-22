package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;


public class Superstructure {
    private SwerveDrive drive;
    private Intake intake;
    private Arm arm;
    private Shooter shooter;
    private Feeder feeder;

    public Superstructure(SwerveDrive drive, Intake intake, Arm arm, Shooter shooter, Feeder feeder) {
        this.drive = drive;
        this.intake = intake;
        this.arm = arm;
        this.shooter = shooter;
        this.feeder = feeder;
    }

    /** Sets the shooter and arm states for a shot. Finishes when the shooter and arm are ready for shooting. */
    public Command setShotState(ShotMap.State state) {
        return parallel(
            shooter.setVelocityC(state),
            arm.setRotationC(state)
        );
    }

    /** 
     * Feeds note into shooter until shot is detected. Consider if a timeout is needed.
     * This may not work for slow speeds.
     */
    public Command feed() {
        return feeder.setVoltageInC().until(shooter::isShotDetected).finallyDo(()->feeder.setVoltage(0));
    }

    public Command intake(){
        return parallel(
            intake.setVoltageC(8,6),
            feeder.setVoltageInC()
        ).until(()->intake.isStalled()).andThen(()->{
            intake.setVoltage(6,4);
            feeder.setVoltageInC();
        }).until(()->feeder.isNoteSensed()).andThen(
            parallel(
                feeder.setVoltageC(-1),
                shooter.setVoltageC(-2, -2),
                idle()
            ).until(()->!feeder.isNoteSensed())
        ).finallyDo(()->{
            intake.setVoltage(0, 0);
            feeder.setVoltage(0);
            shooter.setVoltage(0, 0);
        });
    }

    public Command outtake(){
        return parallel(
            intake.setVoltageOutC(),
            feeder.setVoltageOutC()
        ).finallyDo(()->{
            intake.setVoltage(0,0);
            feeder.setVoltage(0);
        });
    }

    /** Shoots subwoofer. Ends when shot is detected. */
    public Command shootSubwoof(){
        return sequence(
            // setShotState(ShotMap.kSubwoofer),
            parallel( // we can ignore some waiting by just setting voltages
                shooter.setVoltageC(9, 8),
                // arm.setRotationC(ShotMap.kSubwoofer),
                waitSeconds(1)
            ),
            // feed(),
            feeder.setVoltageC(7),
            idle()//.until(()->shooter.isShotDetected()).withTimeout(2),
            // waitSeconds(0.2),
            // shooter.stopC()
        );
    }
}
