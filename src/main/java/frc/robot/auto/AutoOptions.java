package frc.robot.auto;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoOptions {
    public SendableChooser<Command> autoOptions = new SendableChooser<Command>();
    private SwerveDrive drive;
    private Intake intake;
    private Arm arm;
    private Shooter shooter;
    private Feeder feeder;
    private Superstructure superstructure;

    private boolean autosSetup = false;

    public AutoOptions(SwerveDrive drive, Intake intake, Arm arm, Shooter shooter, Feeder feeder, Superstructure superstructure) {
        this.drive = drive;
        this.intake = intake;
        this.arm = arm;
        this.shooter = shooter;
        this.feeder = feeder;
        this.superstructure = superstructure;

        SmartDashboard.putData(autoOptions);

        AutoBuilder.configureHolonomic(
            ()->drive.getPose(),
            (resetPose)->drive.resetOdometry(resetPose),
            ()->drive.getChassisSpeeds(),
            (targetChassisSpeeds)->drive.setChassisSpeeds(targetChassisSpeeds, false, true),
            AutoConstants.kPathConfig,
            ()->drive.getIsDrivingMirrored(),
            drive
        );

        addAutoMethods();
    }

    private Command resetInitialOdomC() {
        return runOnce(()->{
            Rotation2d initialRot = new Rotation2d();
            if(drive.getIsDrivingMirrored()){
                initialRot = new Rotation2d(Math.PI);
            }
            drive.resetOdometry(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    initialRot
                )
            );
        });
    }

    public void periodic(){
        if (!autosSetup && !DriverStation.getAlliance().isEmpty()){
            autoOptions.setDefaultOption("none", resetInitialOdomC());
            addDriveOnlyOptions();
            addArmlessShooterOptions();
            autosSetup = true;
        }
    }

    public void addDriveOnlyOptions(){
        autoOptions.addOption("AmpTaxi",
            AutoBuilder.buildAuto("AmpTaxi")
        );
        autoOptions.addOption("CenterTaxi",
            AutoBuilder.buildAuto("CenterTaxi")
        );
        autoOptions.addOption("SourceTaxi",
            AutoBuilder.buildAuto("SourceTaxi")
        );
    }

    public void addArmlessShooterOptions(){
        autoOptions.addOption("Amp Side Subwoofer 2n",
            AutoBuilder.buildAuto("Amp Side Subwoofer 2n")
        );
        autoOptions.addOption("Subwoofer Front 2n",
            AutoBuilder.buildAuto("Subwoofer Front 2n")
        );
        autoOptions.addOption("Source Side Subwoofer 2n",
            AutoBuilder.buildAuto("Source Side Subwoofer 2n")
        );
        autoOptions.addOption("Shoot1Note", resetInitialOdomC().andThen(superstructure.shootSubwoof().withTimeout(4)));
    }

    public Command getAuto(){
        return Optional.ofNullable(autoOptions.getSelected()).orElse(none());
    }

    private void addAutoMethods(){
        NamedCommands.registerCommand("Intake", superstructure.intake());
        NamedCommands.registerCommand("ShootSubwoofer", superstructure.shootSubwoof().withTimeout(1));
    }
}
