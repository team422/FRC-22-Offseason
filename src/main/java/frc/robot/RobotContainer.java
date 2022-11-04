// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoTestSequence;
import frc.robot.commands.Drive;
import frc.robot.subsystems.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
    private final SwerveModule mTest;
    private final SwerveModule mTest2;
    private XboxController myController;
    // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        mTest = new SwerveModule(Constants.DriveConstants.kSwerveTestMotorDrive,
                Constants.DriveConstants.kSwerveTestMotorTurning, Constants.DriveConstants.analogEncoderSwerveTesting,
                0);
        mTest2 = new SwerveModule(Constants.DriveConstants.kSwerveTestMotorDrive2, // SET REAL CONSTANT VALUES
                Constants.DriveConstants.kSwerveTestMotorTurning2, Constants.DriveConstants.analogEncoderSwerveTesting2,
                1);
        configureButtonBindings();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        myController = new XboxController(0);
        new JoystickButton(myController, 1).whenHeld(new Drive(mTest, () -> myController.getLeftX(),
                () -> myController.getLeftY(), () -> myController.getRightX()));
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
        return new AutoTestSequence(mTest2, mTest, 0.2);
        // return null;
    }
}
