// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;

public class RobotContainer {

    private GenericEntry sb_matchTimer;

    private final Drivetrain drivetrain;

    private final Elevator elevator;

    // private final Intake intake;
    
    //private final ColorSensor colorSensor;

    //private final Vision vision;

    // private final Indexer indexer;

    //private final EndEffector endEffector;

    private final CommandXboxController driverCtrl;

    // private final CommandXboxController operatorCtrl;

    private final SendableChooser<Command> autonChooser;

    private final MutVoltage leftCtrlVolt = Volts.mutable(0);
    private final MutVoltage rightCtrlVolt = Volts.mutable(0);
    private final MutVoltage intakeCtrlVolt = Volts.mutable(0);

    /**
     * Sets up the robot
     */
    public RobotContainer() {

        // Adding match timer printout TODO remove if this does not work
        var match_info = Shuffleboard.getTab("Occra 2025")
            .getLayout("Match Info", BuiltInLayouts.kList)
            .withSize(2, 4);

        sb_matchTimer = match_info.add("Match Timer", -1).getEntry();


        // Initialize Robot
        drivetrain = new Drivetrain(Constants.lfDriveMotorID, Constants.lbDriveMotorID, Constants.rfDriveMotorID,
                Constants.rbDriveMotorID, Constants.driveRatio, Constants.wheelDiameter);

        // indexer = new Indexer(Constants.topIndexerMotorID, Constants.botIndexMotorID);
        // //colorSensor = new ColorSensor();

        elevator = new Elevator(Constants.elevatorMotorID);

        // intake = new Intake(Constants.lIntakeMotorID, Constants.rIntakeMotorID);

        //endEffector = new EndEffector(Constants.lEndEffectorMotorID, Constants.rEndEffectorMotorID);
        //vision = new Vision("Microsoft_LifeCam_HD-3000");

        // Initialize Controls
        driverCtrl = new CommandXboxController(0);
        // operatorCtrl = new CommandXboxController(1);

        // Initialize auton chooser
        autonChooser = AutoBuilder.buildAutoChooser();
        autonChooser.setDefaultOption("None", Commands.print("No autonomous command configured"));
        // autonChooser.addOption("Drive Forward", drivetrain.getDriveDistCmd(Feet.of(3),  Volts.of(6)));
        // autonChooser.addOption("Drive-Turn-Drive", getDriveTurnDriveAuto());
        autonChooser.addOption("Drive Forward Auto", getDriveForwardAuto());
        autonChooser.addOption("Test Left", drivetrain.getDriveLDistanceCmd(Volts.of(-3), Meter.of(0.5)));
        autonChooser.addOption("SysIdRoutine", drivetrain.getSysIdCommandGroup());
        
        // Configure control bindings
        configureBindings();

        SmartDashboard.putData("Auton Chooser", autonChooser);
    }

    /**
     * Setup command triggers and bindings
     */
    private void configureBindings() {
        // Map Driver controls
        drivetrain.setDefaultCommand(
            drivetrain.getDriveCmd(
                () -> leftCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getLeftY(), .1) * Constants.driveVolt, Volts), 
                () -> rightCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getRightY(), .1) * Constants.driveVolt, Volts)
        ));

        driverCtrl.axisGreaterThan(1, 0.1).or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1).onTrue(
            drivetrain.getDriveCmd(
                () -> leftCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getLeftY(), .1) * Constants.driveVolt, Volts), 
                () -> rightCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getRightY(), .1) * Constants.driveVolt, Volts)
        ));

        // driverCtrl.axisGreaterThan(3, 0.1).whileTrue(endEffector.getIndEndEffectorCmd(() -> Volts.of(Constants.lEndEffectorVolt), () -> Volts.of(Constants.rEndEffectorVolt)));
        // driverCtrl.axisGreaterThan(2, 0.1).whileTrue(endEffector.getIndEndEffectorCmd(() -> Volts.of(-Constants.lEndEffectorVolt), () -> Volts.of(-Constants.rEndEffectorVolt)));

        // driverCtrl.leftBumper().onTrue(endEffector.getIndEndEffectorCmd(() -> Volts.of(3), () -> Volts.of(0)));
        // driverCtrl.rightBumper().onTrue(endEffector.getIndEndEffectorCmd(() -> Volts.of(0), () -> Volts.of(3)));

        // driverCtrl.a().onTrue(drivetrain.getDrivePosCmd(Meters.of(1), Meters.of(1)));
        // driverCtrl.b().onTrue(drivetrain.getDriveRateCmd(MetersPerSecond.of(1), MetersPerSecond.of(1)));

        //driverCtrl.axisGreaterThan(3, 0.1).whileTrue(elevator.getElevVoltCmd(() -> Volts.of(driverCtrl.getRightTriggerAxis() * 6)));
        
        // driverCtrl.y().whileTrue(intake.getIndIntakeCmd(() -> Volts.of(-6), () -> Volt.of(6))); //Intake In
        // driverCtrl.x().whileTrue(intake.getIndIntakeCmd(() -> Volts.of(6), () -> Volt.of(-6))); //Intake Out

        // operatorCtrl.leftBumper().whileTrue(indexer.getIndIndexCmd(() -> Volts.of(-Constants.indexVolt), () -> Volt.of(Constants.indexVolt))
        //     .alongWith(intake.getIntakeCmd(() -> Volts.of(Constants.intakeVolt))));

        // operatorCtrl.rightBumper().whileTrue(indexer.getIndIndexCmd(() -> Volts.of(Constants.topIntakeVolt), () -> Volts.of(-Constants.botIntakeVolt))
        //     .alongWith(intake.getIntakeCmd(() -> Volts.of(-Constants.intakeVolt)))); //Index Reverse

        // operatorCtrl.povUp().whileTrue(indexer.getIndIndexCmd(() -> Volts.of(6), () -> Volt.of(6))); //Index In

        // operatorCtrl.povLeft().whileTrue(intake.getIndIntakeCmd(() -> Volts.of(6), () -> Volt.of(3)));

        // operatorCtrl.povRight().whileTrue(intake.getIndIntakeCmd(() -> Volts.of(3), () -> Volt.of(6)));

        // operatorCtrl.axisGreaterThan(3, 0.1).whileTrue(endEffector.getEndEffectorCmd(() -> Volts.of(12)));

        // operatorCtrl.axisGreaterThan(2, 0.1).whileTrue(endEffector.getEndEffectorCmd(() -> Volts.of(-12)));

        driverCtrl.x().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(2.8)));
        driverCtrl.y().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(8.1)));
        driverCtrl.b().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(12.7)));
        driverCtrl.a().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(0)));

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


    private Command getDriveForwardAuto(){
        return Commands.sequence(
            Commands.race(
                drivetrain.getDriveCmd(() -> Volts.of(3), () -> Volts.of(3)),
                Commands.waitSeconds(1.5)
                )
        );
    }


}
