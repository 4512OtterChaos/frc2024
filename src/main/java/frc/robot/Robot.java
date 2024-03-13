// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutoOptions;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.OCXboxController;

public class Robot extends TimedRobot {
    private SwerveDrive swerve = new SwerveDrive();
    private Climber climber = new Climber();
    private Intake intake = new Intake();
    private Feeder feeder = new Feeder();
    // private Arm arm = new Arm();
    private Shooter shooter = new Shooter();

    private Vision vision = new Vision();

    private Superstructure superstructure = new Superstructure(swerve, intake, shooter, feeder);

    private AutoOptions autos = new AutoOptions(swerve, intake, shooter, feeder, superstructure);
    
    private OCXboxController driver = new OCXboxController(0);
    private OCXboxController operator = new OCXboxController(1);

    @Override
    public void robotInit() {
        configureDriverBinds(driver);
        // configureOperatorBinds(operator);

        //TODO: plug usb drive into roborio
        SignalLogger.start();

        climber.setDefaultCommand(climber.holdPositionC());
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        autos.periodic();
    }
    
    @Override
    public void autonomousInit() {
        autos.getAuto().schedule();
    }
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {}
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void disabledInit() {}
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void testInit() {}
    
    @Override
    public void testPeriodic() {}

    //----- Bindings

    private void configureDriverBinds(OCXboxController controller){
        //TODO: use real max drivespeed meters/second
        swerve.setDefaultCommand(
            run(()->{
                    int invert=1;
                    if (swerve.getIsDrivingMirrored()){
                        invert=-1;
                    }
                    swerve.drive(
                        controller.getForward()*swerve.getMaxLinearVelocityMeters()*invert,
                        controller.getStrafe()*swerve.getMaxLinearVelocityMeters()*invert,
                        controller.getTurn()*swerve.getMaxAngularVelocityRadians(),
                        true
                    );
                },
                swerve
            )
        );
    
        // controller.rightTrigger().whileTrue(shooter.CShootTable(2));
        // controller.b().whileTrue(shooter.CShootAmp());
        // controller.x().onTrue(arm.CSetAngle(0));

        // controller.povUp().onTrue(climber.CSetMaxHeight());
        controller.povDown().onTrue(climber.CSetMinHeight());

        controller.b()
            .whileTrue(superstructure.intake());
        controller.leftStick()
            .whileTrue(superstructure.outtake());        
        controller.a().whileTrue(superstructure.shootSubwoof());

        controller.leftBumper()
            .whileTrue(climber.setLeftVoltageUpC())
            .onFalse(climber.setLeftVoltageC(0));
        controller.rightBumper()
            .whileTrue(climber.setRightVoltageUpC())
            .onFalse(climber.setRightVoltageC(0));

        controller.leftTrigger()
            .whileTrue(climber.setLeftVoltageDownC())
            .onFalse(climber.setLeftVoltageC(0));
        controller.rightTrigger()
            .whileTrue(climber.setRightVoltageDownC())
            .onFalse(climber.setRightVoltageC(0));


        // reset the robot heading to 0
        controller.start()
            .onTrue(runOnce(()->
                swerve.resetOdometry(
                    new Pose2d(
                        swerve.getPose().getTranslation(),
                        new Rotation2d()
                    )
                )
            )
        );
    }

    private void configureOperatorBinds(OCXboxController controller) {
        // controller.povUp().onTrue(climber.CSetMaxHeight());
        controller.povDown().onTrue(climber.CSetMinHeight());

        controller.leftBumper().whileTrue(climber.setLeftVoltageUpC());
        controller.rightBumper().whileTrue(climber.setRightVoltageUpC());

        controller.leftTrigger().whileTrue(climber.setLeftVoltageDownC());
        controller.rightTrigger().whileTrue(climber.setRightVoltageDownC());
    }


    //----- Simulation

    @Override
    public void simulationInit() {}
    
    @Override
    public void simulationPeriodic() {
        // Update camera simulation
        // TODO: dont use the pose estimator pose
        vision.simulationPeriodic(swerve.getPose());

        // Calculate battery voltage sag due to current draw
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(swerve.getCurrentDraw()));
    }
}
