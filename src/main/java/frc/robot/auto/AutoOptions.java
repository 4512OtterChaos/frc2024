package frc.robot.auto;

import static frc.robot.subsystems.drive.SwerveConstants.kSwerveCenterRadius;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class AutoOptions {
    public SendableChooser<Command> autoOptions = new SendableChooser<Command>();
    private SwerveDrive drive;
    private Intake intake;
    private Shooter shooter;
    private Feeder feeder;
    private Superstructure superstructure;
    private boolean autosSetup = false;

    public AutoOptions(SwerveDrive drive, Intake intake, Shooter shooter, Feeder feeder, Superstructure superstructure) {
        this.drive=drive;
        this.intake = intake;
        this.shooter = shooter;
        this.feeder = feeder;
        this.superstructure = superstructure;

        SmartDashboard.putData(autoOptions);
        addAutoMethods();
        
        // autoOptions.setDefaultOption("none", null);
        // addDriveOnlyOptions();
    }
    public AutoOptions(SwerveDrive drive) {
        // this.drive=drive;
        // autoOptions.setDefaultOption("none", null);
        // addDriveOnlyOptions();
    }

    public void autoInit(){
        if (autosSetup==false){
            autoBuilderConfig(drive);
            autoOptions.setDefaultOption("none",  
                run(()->{
                    double gyroRotation = 0;
                    if(drive.flipAutoOrgin()){
                        gyroRotation = 180;
                    }
                    drive.resetOdometry(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            new Rotation2d(gyroRotation)
                        )
                    );
                })
            );
            addDriveOnlyOptions();
            autosSetup=true;
        }
    }

    public void robotPeriodic(){
        
        if ((!DriverStation.getAlliance().isEmpty())&&(autosSetup==false)){
            autoBuilderConfig(drive);
            autoOptions.setDefaultOption("none",  
                run(()->{
                    double gyroRotation = 0;
                    if(drive.flipAutoOrgin()){
                        gyroRotation = 180;
                    }
                    drive.resetOdometry(
                        new Pose2d(
                            drive.getPose().getTranslation(),
                            new Rotation2d(gyroRotation)
                        )
                    );
                })
            );
            addDriveOnlyOptions();

            autoOptions.addOption("Shoot1Note", superstructure.shootSubwoof().withTimeout(4));
            autosSetup=true;
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

    public Command getAuto(){
        return Optional.ofNullable(autoOptions.getSelected()).orElse(none());
    }

    public HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig(Units.feetToMeters(7), kSwerveCenterRadius, null);
    
    private void autoBuilderConfig(SwerveDrive drive){
        AutoBuilder.configureHolonomic(
            ()->drive.getPose(),
            (resetPose)->drive.resetOdometry(resetPose),
            ()->drive.getChassisSpeeds(),
            (targetChassisSpeeds)->drive.setChassisSpeeds(targetChassisSpeeds, false, true),
            pathConfig,
            ()->drive.flipAutoOrgin(),
            drive
        );
    }

    private void addAutoMethods(){
        NamedCommands.registerCommand("Intake", superstructure.intake());
        NamedCommands.registerCommand("ShootSubwoofer", superstructure.shootSubwoof());
    }

    // private void addAutoMethods(){
    //     AutoBuilder.
    // }
}
