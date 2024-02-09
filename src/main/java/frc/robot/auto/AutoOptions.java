package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superShooter.arm.Arm;
import frc.robot.subsystems.superShooter.feeder.Feeder;
import frc.robot.subsystems.superShooter.shooter.Shooter;

public class AutoOptions {
    public SendableChooser<Command> autoOptions = new SendableChooser<Command>();

    public AutoOptions(Drivetrain drivetrain, Intake intake, Arm arm, Shooter shooter, Feeder feeder){
        autoOptions.setDefaultOption("Hold still", null);
    }

    public Command getAuto(){
        return autoOptions.getSelected();
    }
}
