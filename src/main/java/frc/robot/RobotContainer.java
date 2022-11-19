// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.FullSwerveDrive;
import frc.robot.commands.Turn;
import frc.robot.subsystems.FullSwerveBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.MotorFactory;
import frc.robot.utils.SubsystemFactory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    // private final SwerveModule mTest;
    // private final SwerveModule mTest2;
    SwerveModule m_RightFrontSwerveModule;
    SwerveModule m_LeftFrontSwerveModule;
    SwerveModule m_RightRearSwerveModule;
    SwerveModule m_LeftRearSwerveModule;
    SwerveModule[] m_SwerveModules;

    FullSwerveBase m_SwerveBase;

    private XboxController myController;
    private SubsystemFactory subsystemFactory;

    ADXRS450_Gyro m_Gyro;

    // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer(SubsystemFactory subsystemFactory) {
        this.subsystemFactory = subsystemFactory;
        m_SwerveBase = this.subsystemFactory.createFullSwerveBase();

        configureButtonBindings();
    }

    public RobotContainer() {
        MotorFactory motorFactory = new MotorFactory();
        this.subsystemFactory = new SubsystemFactory(motorFactory);
        m_SwerveBase = this.subsystemFactory.createFullSwerveBase();
    }

    public void printDriveBaseVals() {
        m_SwerveBase.printAllVals();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        myController = new XboxController(0);
        // new JoystickButton(myController, 1).whenHeld(new DriveOneModule(mTest, () -> myController.getLeftX(),
        //         () -> myController.getLeftY(), () -> myController.getRightX()));
        FullSwerveDrive driveCommand = new FullSwerveDrive(m_SwerveBase, () -> -myController.getLeftX(),
                () -> myController.getLeftY(), () -> -myController.getRightX());
        m_SwerveBase.setDefaultCommand(driveCommand);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return m_autoCommand;
        // mTest.setDesiredState(Double);
        // return new AutoTestSequence(mTest2, mTest, 0.2);
        return new Turn(m_SwerveBase, 50);
        // return null;
    }
}
