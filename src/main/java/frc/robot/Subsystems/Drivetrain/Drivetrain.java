package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;


public class Drivetrain extends SubsystemBase {
    SwerveModule[] swerveMods = {
        new SwerveModule(FL, getName()),
        new SwerveModule(FR, getName()),
        new SwerveModule(BL, getName()),
        new SwerveModule(BR, getName())
    };
    SwerveControllerCommand drivebase = new SwerveControllerCommand(null, null, null, null, null, this);
}
