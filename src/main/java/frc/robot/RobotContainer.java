// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
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

    private final EndEffector endEffector;

    //private final EndEffector endEffector;

    private final CommandXboxController driverCtrl;

    private final CommandXboxController operatorCtrl;

    private final SendableChooser<Command> autonChooser;

    private final MutVoltage leftCtrlVolt = Volts.mutable(0);
    private final MutVoltage rightCtrlVolt = Volts.mutable(0);

    /**
     * Sets up the robot
     */
    public RobotContainer() {

        // Adding match timer printout TODO remove if this does not work
        var match_info = Shuffleboard.getTab("Occra 2025")
            .getLayout("Match Info", BuiltInLayouts.kList)
            .withSize(2, 4);

        sb_matchTimer = match_info.add("Match Timer", -1).getEntry();


        // Initialize Robot & set motor IDs
        drivetrain = new Drivetrain(Constants.lfDriveMotorID, Constants.lbDriveMotorID, Constants.rfDriveMotorID,
                Constants.rbDriveMotorID, Constants.driveRatio, Constants.wheelDiameter);
        
        elevator = new Elevator(Constants.elevatorMotorID);
        
        endEffector = new EndEffector(Constants.lEndEffectorMotorID, Constants.rEndEffectorMotorID);

        // Initialize Controls
        driverCtrl = new CommandXboxController(0);
        operatorCtrl = new CommandXboxController(1);

        // Initialize auton chooser
        autonChooser = AutoBuilder.buildAutoChooser();
        autonChooser.setDefaultOption("None", Commands.print("No autonomous command configured"));
        // autonChooser.addOption("Drive Forward", drivetrain.getDriveDistCmd(Feet.of(3),  Volts.of(6)));
        // autonChooser.addOption("Drive-Turn-Drive", getDriveTurnDriveAuto());
        autonChooser.addOption("Drive Forward Auto", getDriveForwardAuto());
        autonChooser.addOption("Test Left", drivetrain.getDriveLDistanceCmd(Volts.of(-3), Meter.of(0.5)));
        autonChooser.addOption("SysIdRoutine", drivetrain.getSysIdCommandGroup());
        autonChooser.addOption("Test Auton", getTestAuto());
        autonChooser.addOption("End Effector SysID", endEffector.sysIdCommandGroup);
        autonChooser.addOption("One Block Auto", getOneBlockAuto());
        
        // Configure control bindings
        configureBindings();

        SmartDashboard.putData("Auton Chooser", autonChooser);
    }

    /**
     * @return Setup command triggers and bindings
     */
    private void configureBindings() {
        // // Maps default Driver controls
        // drivetrain.setDefaultCommand(
        //     drivetrain.getDriveCmd(
        //         () -> leftCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getLeftY(), .1) * Constants.driveVolt, Volts), 
        //         () -> rightCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getRightY(), .1) * Constants.driveVolt, Volts)
        // ));

        // // Maps override Driver controls
        // driverCtrl.axisGreaterThan(1, 0.1).or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1).onTrue(
        //     drivetrain.getDriveCmd(
        //         () -> leftCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getLeftY(), .1) * Constants.driveVolt, Volts), 
        //         () -> rightCtrlVolt.mut_replace(MathUtil.applyDeadband(-driverCtrl.getRightY(), .1) * Constants.driveVolt, Volts)
        // ));

        // driverCtrl.axisGreaterThan(1, 0.1).or(() -> Math.abs(driverCtrl.getRightY()) >= 0.1)
        //     .onTrue(
        //         drivetrain.getTankDriveCmd(() -> driverCtrl.getLeftY(), () -> driverCtrl.getRightY())
        // );

        driverCtrl.axisGreaterThan(1, 0.1).or(() -> Math.abs(driverCtrl.getRightX()) >= 0.1)
            .onTrue(
                drivetrain.getArcadeDriveCmd(() -> -MathUtil.applyDeadband(driverCtrl.getLeftY(), 0.1), () -> -MathUtil.applyDeadband(driverCtrl.getRightX(), 0.1))
        );

        drivetrain.setDefaultCommand(drivetrain.getArcadeDriveCmd(() -> -MathUtil.applyDeadband(driverCtrl.getLeftY(), 0.1), () -> -MathUtil.applyDeadband(driverCtrl.getRightX(), 0.1)));

        operatorCtrl.axisGreaterThan(3, 0.1).whileTrue(endEffector.getIntakeCmd());
        operatorCtrl.axisGreaterThan(2, 0.1).whileTrue(endEffector.getIndEndEffectorCmd(() -> Constants.lEndEffectorVolt.times(-1), () -> Constants.rEndEffectorVolt.times(-1)));

        operatorCtrl.leftBumper().whileTrue(endEffector.getIndEndEffectorCmd(() -> Volts.of(-12), () -> Volts.of(-2)));
        operatorCtrl.rightBumper().whileTrue(endEffector.getIndEndEffectorCmd(() -> Volts.of(-2), () -> Volts.of(-12)));

        // driverCtrl.a().onTrue(drivetrain.getDrivePosCmd(Meters.of(1), Meters.of(1)));
        // driverCtrl.b().onTrue(drivetrain.getDriveRateCmd(MetersPerSecond.of(1), MetersPerSecond.of(1)));

        operatorCtrl.x().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(6.9))); //Elevator to shelf 1
        operatorCtrl.y().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(12.2))); //Elevator to shelf 2
        operatorCtrl.b().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(16.7))); //Elevator to shelf 3
        operatorCtrl.a().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(0))); //Elevator to bottom

        operatorCtrl.povLeft().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(8.4)));
        operatorCtrl.povUp().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(13.8)));
        operatorCtrl.povRight().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(16.7)));
        operatorCtrl.povDown().onTrue(elevator.getElevatorPosCmd(() -> Rotations.of(1.3)));

        driverCtrl.a().whileTrue(drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.zero()));

    }

    /**
     * 
     * @return returns the auton chooser
     */
    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }

    /**
     * 
     * @return Auton to drive forwards for 1.5 seconds with a voltage of 3
     */
    private Command getDriveForwardAuto(){
        return Commands.sequence(
            Commands.race(
                drivetrain.getDriveCmd(() -> Volts.of(3), () -> Volts.of(3)),
                Commands.waitSeconds(1.5)
                )
        );
    }

    private Command getTestAuto(){
        return Commands.sequence(
            drivetrain.getDriveDistanceCmd(Volts.of(3), Meters.of(3.5), Meters.of(3.5))
        );
    }

    private Command getOneBlockAuto(){
        return Commands.sequence(
            Commands.race(
                drivetrain.getDriveDistanceCmd(Volts.of(5), Meters.of(3.8), Meters.of(3.8)),
                endEffector.getIntakeCmd()
            ),

            elevator.getElevatorPosCmd(() -> Rotations.of(1.3)).deadlineFor(Commands.waitSeconds(0.5)),

            Commands.race(
                drivetrain.getDriveToAnglePIDCmd(Degrees.of(90), Degrees.of(1))
            ),
            Commands.race(
                drivetrain.getDriveDistanceCmd(Volts.of(3), Meters.of(0.2), Meters.of(0.2)),
                elevator.getElevatorPosCmd(() -> Rotations.of(6.9))
            ),
            Commands.deadline(Commands.waitSeconds(3), 
                elevator.getElevatorPosCmd(() -> Rotations.of(6.9)),
                endEffector.getScoreCmd()
            )
        );
    }


}