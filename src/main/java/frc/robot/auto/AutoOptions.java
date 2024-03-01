package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoOptions {
    public SendableChooser<Command> autoOptions = new SendableChooser<Command>();

    public AutoOptions(SwerveDrive drive, Intake intake, Arm arm, Shooter shooter, Feeder feeder) {
        autoOptions.setDefaultOption("none", null);
        addDriveOnlyOptions();
    }
    public AutoOptions(SwerveDrive drive) {
        autoOptions.setDefaultOption("none", null);
        addDriveOnlyOptions();
    }

    public void addDriveOnlyOptions(){
        // autoOptions.addOption("AmpTaxi",
        //     new OCSwerveFollower(
        //         drive, 
        //         "AmpTaxi", 
        //         AutoConstants.kMediumSpeedConfig,
        //         true
        //     )
        // );
        // autoOptions.addOption("CenterTaxi",
        //     new OCSwerveFollower(
        //         drive, 
        //         "CenterTaxi", 
        //         AutoConstants.kMediumSpeedConfig,
        //         true
        //     )
        // );
        // autoOptions.addOption("SourceTaxi",
        //     new OCSwerveFollower(
        //         drive, 
        //         "SourceTaxi", 
        //         AutoConstants.kMediumSpeedConfig,
        //         true
        //     )
        // );
    }
    public Command getAuto(){
        return Optional.ofNullable(autoOptions.getSelected()).orElse(Commands.none());
    }
}
