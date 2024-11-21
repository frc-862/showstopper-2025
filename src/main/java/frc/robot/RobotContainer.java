// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.thunder.LightningContainer;
import frc.robot.Constants.TunerConstants;

public class RobotContainer extends LightningContainer {

    private XboxController driver;

    private Swerve drivetrain;
    private PhotonVision vision;
    private Telemetry logger;

    private SwerveRequest.FieldCentric driveFieldCentric;
    private SwerveRequest.RobotCentric driveRobotCentric;

    @Override
    protected void initializeSubsystems() {
        this.driver = new XboxController(0);

        this.drivetrain = getDrivetrain();
        this.vision = new PhotonVision();

        this.logger = new Telemetry(drivetrain.getMaxSpeed());

        this.driveFieldCentric = new SwerveRequest.FieldCentric();
        this.driveRobotCentric = new SwerveRequest.RobotCentric();
    }

    @Override
    protected void configureButtonBindings() {
        // new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d).whileTrue(
        // drivetrain.applyPercentRequestRobot(
        // () -> -driver.getLeftY(),
        // () -> -driver.getLeftX(),
        // () -> -driver.getRightX()));

        new Trigger(() -> driver.getLeftTriggerAxis() > 0.25d).whileTrue(
                drivetrain.applyRequest(
                        () -> driveRobotCentric.withRotationalRate(-driver.getRightX() * drivetrain.getMaxAngularRate())
                                .withVelocityX(-driver.getLeftY() * drivetrain.getMaxSpeed())
                                .withVelocityY(-driver.getLeftX() * drivetrain.getMaxSpeed())));

        new Trigger(() -> driver.getRightTriggerAxis() > 0.25d)
                .onTrue(drivetrain.enableSlowMode())
                .onFalse(drivetrain.disableSlowMode());

        new Trigger(() -> driver.getStartButton() && driver.getBackButton()).onTrue(drivetrain.resetForward());

        new Trigger(driver::getXButton).onTrue(drivetrain.setBrake());

        // if (Utils.isSimulation()) {
        // drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
        // Rotation2d.fromDegrees(90)));
        // }
    }

    @Override
    protected void configureDefaultCommands() {
        // drivetrain.setDefaultCommand(
        //         drivetrain.applyPercentRequestField(() -> -(driver.getLeftY() * drivetrain.getSpeedMult()),
        //                 () -> -(driver.getLeftX() * drivetrain.getSpeedMult()),
        //                 () -> -(driver.getRightX() * drivetrain.getRotMult())));
    
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(
                () -> driveFieldCentric.withRotationalRate(-driver.getRightX() * drivetrain.getMaxAngularRate())
                    .withVelocityX(-driver.getLeftY() * drivetrain.getMaxSpeed())
                    .withVelocityY(-driver.getLeftX() * drivetrain.getMaxSpeed())));

        vision.setDefaultCommand(vision.updateOdometry(drivetrain));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @Override
    protected void initializeNamedCommands() {
    }

    @Override
    protected void configureSystemTests() {
    }

    @Override
    protected void releaseDefaultCommands() {
    }

    @Override
    protected void initializeDashboardCommands() {
    }

    @Override
    protected void configureFaultCodes() {
    }

    @Override
    protected void configureFaultMonitors() {
    }

    @Override
    protected Command getAutonomousCommand() {
        return null;
    }

    public Swerve getDrivetrain() {
        return drivetrain == null ? TunerConstants.DriveTrain : drivetrain;
    }
}
