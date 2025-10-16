// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    private final Drivetrain drivetrain;
    
    //private final ColorSensor colorSensor;

    private final Vision vision;

    private final CommandXboxController driverCtrl;

    private final SendableChooser<Command> autonChooser;

    private final MutVoltage leftCtrlVolt = Volts.mutable(0);
    private final MutVoltage rightCtrlVolt = Volts.mutable(0);

    /**
     * Sets up the robot
     */
    public RobotContainer() {
        // Initialize Robot
        drivetrain = new Drivetrain(Constants.lfDriveMotorID, Constants.lrDriveMotorID, Constants.rfDriveMotorID,
                Constants.rrDriveMotorID, Constants.driveRatio, Constants.wheelDiameter);

        //colorSensor = new ColorSensor();

        vision = new Vision("Microsoft_LifeCam _HD-3000");

        // Initialize Controls
        driverCtrl = new CommandXboxController(0);

        // Initialize auton chooser
        autonChooser = new SendableChooser<>();
        autonChooser.setDefaultOption("None", Commands.print("No autonomous command configured"));
        autonChooser.addOption("Drive Forward", drivetrain.getDriveDistCmd(Feet.of(3),  Volts.of(6)));
        autonChooser.addOption("Drive-Turn-Drive", getDriveTurnDriveAuto());
        
        // Configure control bindings
        configureBindings();
    }

    /**
     * Setup command triggers and bindings
     */
    private void configureBindings() {
        // Map Driver controls
        drivetrain.setDefaultCommand(
            drivetrain.getDriveCmd(
                () -> leftCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getLeftY(), .1) * 12, Volts), 
                () -> rightCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getRightY(), .1) * 12, Volts)));

        driverCtrl.a().onTrue(drivetrain.getDriveRMotorCmd(() -> Volts.of(6)));
        driverCtrl.b().onTrue(drivetrain.getDriveLMotorCmd(() -> Volts.of(6)));
        driverCtrl.x().onTrue(drivetrain.getDriveBMotor(() -> Volts.of(12)));
        driverCtrl.y().onTrue(drivetrain.getDriveBMotor(() -> Volts.of(0)));
    }



    /**
     * Gets the currently selected auton
     * @return
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /**
     * Creates auton that drives forward, turns left, and then drives forward
     * @return
     */
    private Command getDriveTurnDriveAuto() {
        return Commands.sequence(
            drivetrain.getDriveDistCmd(Feet.of(3),  Volts.of(6)),
            drivetrain.getTurnTimeCmd(Seconds.of(3),  Volts.of(3)),
            drivetrain.getDriveDistCmd(Feet.of(3),  Volts.of(6)));
    }
}
