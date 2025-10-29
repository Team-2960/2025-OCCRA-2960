// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import java.io.PrintStream;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    private final Drivetrain drivetrain;

    private final Elevator elevator;

    private final Intake intake;
    
    //private final ColorSensor colorSensor;

    //private final Vision vision;

    private final Indexer indexer;

    private final EndEffector endEffector;

    private final CommandXboxController driverCtrl;

    private final CommandXboxController operatorCtrl;

    private final SendableChooser<Command> autonChooser;

    private final MutVoltage leftCtrlVolt = Volts.mutable(0);
    private final MutVoltage rightCtrlVolt = Volts.mutable(0);
    private final MutVoltage intakeCtrlVolt = Volts.mutable(0);

    /**
     * Sets up the robot
     */
    public RobotContainer() {
        // Initialize Robot
        drivetrain = new Drivetrain(Constants.lfDriveMotorID, Constants.lbDriveMotorID, Constants.rfDriveMotorID,
                Constants.rbDriveMotorID, Constants.driveRatio, Constants.wheelDiameter);

        indexer = new Indexer(Constants.topIndexerMotorID, Constants.botIndexMotorID);
        //colorSensor = new ColorSensor();

        elevator = new Elevator(Constants.elevatorMotorID);

        intake = new Intake(Constants.lIntakeMotorID, Constants.rIntakeMotorID);

        endEffector = new EndEffector(Constants.endEffectorMotorID);
        //vision = new Vision("Microsoft_LifeCam_HD-3000");

        // Initialize Controls
        driverCtrl = new CommandXboxController(0);
        operatorCtrl = new CommandXboxController(1);

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
                () -> leftCtrlVolt.mut_replace(MathUtil.applyDeadband(driverCtrl.getLeftY(), .1) * 9, Volts), 
                () -> rightCtrlVolt.mut_replace(MathUtil.applyDeadband(driverCtrl.getRightY(), .1) * 9, Volts)
        ));

        //driverCtrl.axisGreaterThan(3, 0.1).whileTrue(elevator.getElevVoltCmd(() -> Volts.of(driverCtrl.getRightTriggerAxis() * 6)));
        
        // driverCtrl.y().whileTrue(intake.getIndIntakeCmd(() -> Volts.of(-6), () -> Volt.of(6))); //Intake In
        // driverCtrl.x().whileTrue(intake.getIndIntakeCmd(() -> Volts.of(6), () -> Volt.of(-6))); //Intake Out

        operatorCtrl.leftBumper().whileTrue(indexer.getIndIndexCmd(() -> Volts.of(-6), () -> Volt.of(6))
            .alongWith(intake.getIntakeCmd(() -> Volts.of(6))));

        operatorCtrl.rightBumper().whileTrue(indexer.getIndexCmd(() -> Volts.of(6))
            .alongWith(intake.getIntakeCmd(() -> Volts.of(-6)))); //Index Reverse

        operatorCtrl.povUp().whileTrue(indexer.getIndIndexCmd(() -> Volts.of(6), () -> Volt.of(6))); //Index In

        operatorCtrl.axisGreaterThan(3, 0.1).whileTrue(endEffector.getEndEffectorCmd(() -> Volts.of(12)));

        operatorCtrl.axisGreaterThan(2, 0.1).whileTrue(endEffector.getEndEffectorCmd(() -> Volts.of(-12)));


        // driverCtrl.b().whileTrue(indexer.getIndIndexCmd(() -> Volts.of(-6), () -> Volt.of(0))); //Index Out

        // intake
        // intake.setDefaultCommand(intake.getIntakeCmd(
        //     () -> intakeCtrlVolt.mut_replace(MathUtil.applyDeadband(driverCtrl.getLeftTriggerAxis(), .1) * 12, Volts)
        //  ));


        // driverCtrl.y().whileTrue(elevator.getElevRateCmd(() -> AngularVelocity.ofBaseUnits(5, RotationsPerSecond))); //Elevator
        // driverCtrl.leftBumper().whileTrue(intake.getIntakeCmd(() -> Volts.of(-8))); //Intake push out

     /*/    driverCtrl.leftBumper()
            .whileTrue(
                new ParallelCommandGroup(
                    intake.getIndIntakeCmd(() -> Volts.of(-6), () -> Volts.of(6)),
                    indexer.getIntakeCmd(() -> Volts.of(6))
            )
        );

        //INTAKE + INDEXER
        driverCtrl.rightBumper()
            .whileTrue(
                new ParallelCommandGroup(
                    intake.getIndIntakeCmd(() -> Volts.of(6), () -> Volts.of(-6)),
                    indexer.getIntakeCmd(() -> Volts.of(-6))
            )
        );

        driverCtrl.y().whileTrue(intake.getIndIntakeCmd(() -> Volts.of(6), () -> Volt.of(6))); //INTAKE
        /* */
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
