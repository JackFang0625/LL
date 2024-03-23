// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Vision.Limelight;
import frc.robot.commands.autoAlign;
import frc.robot.commands.intakeNote;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.RightClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LeftClimberSubsystem;
import frc.robot.subsystems.LeftFlyWheelSubsystem;
import frc.robot.subsystems.RightFlyWheelSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class RobotContainer {
    private SendableChooser<Command> autoChooser;
    private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
    private final double TurtleSpeed = 0.1; // Reduction in speed from Max Speed, 0.1 = 10%
    private final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    private final double TurtleAngularRate = Math.PI * 0.5; // .75 rotation per second max angular velocity.  Adjust for max turning rate speed.
    private double AngularRate = MaxAngularRate; // This will be updated when turtle and reset to MaxAngularRate

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final CommandPS4Controller driverPS4 = new CommandPS4Controller(0);
    private final CommandPS4Controller operatorPS4 = new CommandPS4Controller(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
    
    private final Limelight limelight = new Limelight(drivetrain);
    private final ArmSubsystem arm = new ArmSubsystem();
    private final LeftClimberSubsystem leftClimber = new LeftClimberSubsystem();
    private final RightClimberSubsystem rightClimber = new RightClimberSubsystem();
    private final IntakeSubsystem intakeWheel = new IntakeSubsystem();
    private final LeftFlyWheelSubsystem leftFlyWheel = new LeftFlyWheelSubsystem();
    private final RightFlyWheelSubsystem rightFlyWheel = new RightFlyWheelSubsystem();
    private final TransferSubsystem  transfer = new TransferSubsystem();
    private final CANdleSubsystem CANdle = new CANdleSubsystem(transfer, intakeWheel, leftFlyWheel, drivetrain, limelight);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                // driving in open loop
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
//    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
//    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    
    Limelight vision = new Limelight(drivetrain);
    Pose2d odomStart = new Pose2d(2.75, 5.55, new Rotation2d(0, 0));
//    private final Telemetry logger = new Telemetry(MaxSpeed);

    private void configureBindings() {
        drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(-driverPS4.getLeftY() * MaxSpeed) // Drive forward with
                                                                                            // negative Y (forward)
                .withVelocityY(-driverPS4.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-driverPS4.getRightX() * AngularRate) // Drive counterclockwise with negative X (left)
            ).ignoringDisable(true));

        driverPS4.square().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on options press
        driverPS4.options().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

        driverPS4.L1().onTrue(runOnce( () -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * TurtleSpeed)
        .andThen( () -> AngularRate = TurtleAngularRate) );
        driverPS4.L1().onFalse(runOnce( () -> MaxSpeed = TunerConstants.kSpeedAt12VoltsMps)
        .andThen( () -> AngularRate = MaxAngularRate) );

        // Operator
        operatorPS4.triangle().whileTrue(arm.ampCommand());
        operatorPS4.cross().whileTrue(arm.stowedCommand());
        operatorPS4.circle().whileTrue(arm.sourceCommand());
        operatorPS4.share().whileTrue(new autoAlign(arm, limelight));

        operatorPS4.povDown().whileTrue(arm.runCommand(-0.1));
        operatorPS4.povLeft().whileTrue(arm.speakerCommand());

        leftClimber.runCommand(operatorPS4.getLeftY());
        rightClimber.runCommand(operatorPS4.getRightY());

        operatorPS4.L2().whileTrue(Commands.parallel(leftFlyWheel.pidCommand(3000), rightFlyWheel.pidCommand(2400)));
        //operatorPS4.circle().whileTrue(Commands.parallel(leftFlyWheel.pidCommand(-3000), rightFlyWheel.pidCommand(-3000)));

        operatorPS4.R2().onTrue(transfer.feedNote2ShooterCommand());

        operatorPS4.L1().whileTrue( new intakeNote(intakeWheel, transfer) );
        operatorPS4.R1().whileTrue( intakeWheel.ejectIntakeCommand() );
    }

    public RobotContainer() {
        // Register Named Commands
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    public Command getAutonomousCommand() {
        /* First put the drivetrain into auto run mode, then run the auto */
        return autoChooser.getSelected();
    }
}
