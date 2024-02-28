package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;

import frc.robot.auto.AutoOptions;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.OCXboxController;

public class RobotContainer {
    private SwerveDrive swerve = new SwerveDrive();
    private Climber climber = new Climber();
    private Intake intake = new Intake();
    private Feeder feeder = new Feeder();
    private Shooter shooter = new Shooter();
    private Arm arm = new Arm();

    private AutoOptions autos = new AutoOptions(swerve,intake,arm,shooter,feeder);
    
    private OCXboxController driver = new OCXboxController(0);
    // private OCXboxController operator = new OCXboxController(1);
    
    public RobotContainer(){
        configureDriverBinds(driver);
        // configureOperatorBinds(operator);

        //TODO: plug usb drive into roborio
        SignalLogger.start();
    }

    public Command getAuto(){
        return autos.getAuto();
    }

    private void configureDriverBinds(OCXboxController controller){
        //TODO: use real max drivespeed meters/second
        swerve.setDefaultCommand(
            swerve.cDrivePercent(controller::getForward, controller::getStrafe, controller::getTurn).ignoringDisable(true)
        );
        intake.setDefaultCommand(intake.setVoltageC(0));
        climber.setDefaultCommand(null);
    
        controller.rightTrigger().whileTrue(shooter.CShootTable(2));
        controller.leftTrigger().whileTrue(intake.setVoltageInC());

        controller.leftStick().whileTrue(intake.setVoltageOutC());

        controller.x().onTrue(arm.CSetAngle(0));
        controller.a().whileTrue(shooter.CShootSubwoof());
        controller.b().whileTrue(shooter.CShootAmp());

        controller.povUp().onTrue(climber.CSetMaxHeight());
        controller.povDown().onTrue(climber.CSetMinHeight());

        // reset field-relative forward direction
        controller.start().onTrue(runOnce(()->swerve.seedFieldRelative()));
    }

    private void configureOperatorBinds(OCXboxController controller) {
        /* Bindings for drivetrain characterization */
        /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
        /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
        controller.back().and(controller.y()).whileTrue(swerve.cSysIdDynamic(Direction.kForward));
        controller.back().and(controller.a()).whileTrue(swerve.cSysIdDynamic(Direction.kReverse));
        controller.start().and(controller.y()).whileTrue(swerve.cSysIdQuasistatic(Direction.kForward));
        controller.start().and(controller.a()).whileTrue(swerve.cSysIdQuasistatic(Direction.kReverse));
    }
}
