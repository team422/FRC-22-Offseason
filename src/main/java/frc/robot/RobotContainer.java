// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.climbcommands.TeleClimbDown;
import frc.robot.commands.climbcommands.TeleClimbDownLeft;
import frc.robot.commands.climbcommands.TeleClimbDownRight;
import frc.robot.commands.climbcommands.TeleClimbTilt;
import frc.robot.commands.climbcommands.TeleClimbUp;
import frc.robot.commands.climbcommands.TeleClimbUpLeft;
import frc.robot.commands.climbcommands.TeleClimbUpRight;
import frc.robot.oi.MixedXboxJoystickControls;
import frc.robot.oi.UserControls;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOFalcon;
import frc.robot.subsystems.climber.ClimberPistonIO;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private Climber climber;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Initialize Subsystems
        configureSubsystems();

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureSubsystems() {
        switch (Constants.bot) {
            case ROBOT_2022_COMP:
                climber = new Climber(
                        new ClimberIOFalcon(),
                        new ClimberPistonIO());
                break;
            case ROBOT_2022_PRACTICE:
                break;
            default:
                System.out.println("No robot selected.");
                break;
        }

        climber = climber != null ? climber : new Climber(new ClimberIO() {
        }, null);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        UserControls controls = new MixedXboxJoystickControls(0, 1, 5);

        //defining commands lmao
        TeleClimbUp climberUpCommand = new TeleClimbUp(climber);
        TeleClimbDown climberDownCommand = new TeleClimbDown(climber);
        TeleClimbUpLeft climberUpLeftCommand = new TeleClimbUpLeft(climber);
        TeleClimbDownLeft climberDownLeftCommand = new TeleClimbDownLeft(climber);
        TeleClimbUpRight climberUpRightCommand = new TeleClimbUpRight(climber);
        TeleClimbDownRight climberDownRightCommand = new TeleClimbDownRight(climber);
        TeleClimbTilt climmberTiltCommand = new TeleClimbTilt(climber);

        //binding things
        controls.getClimbUp().whileActiveOnce(climberUpCommand);
        controls.getClimbDown().whileActiveOnce(climberDownCommand);
        controls.getClimbButton().whenActive(climmberTiltCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
