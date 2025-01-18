// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import javax.print.attribute.standard.JobName;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  
  private final CommandXboxController johnController = new CommandXboxController(0);
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   
    //johnController.b().onTrue(m_exampleSubsystem.johnMoveForward()).onFalse(m_exampleSubsystem.johnStop());

    johnController.a().onTrue(m_exampleSubsystem.johnMove10Feet(1.0));

    //johnController.x().whileTrue(m_exampleSubsystem.johnTurnLeft()).onFalse(m_exampleSubsystem.johnStop());

    //johnController.b().whileTrue(m_exampleSubsystem.johnTurnRight()).onFalse(m_exampleSubsystem.johnStop());
    
    m_exampleSubsystem.setDefaultCommand(m_exampleSubsystem.johnMove(()->johnController.getRightX(), ()->johnController.getLeftY()));

    //CommandScheduler.getInstance().setDefaultCommand(m_exampleSubsystem,m_exampleSubsystem.johnMove(()->johnController.getLeftX(), ()->johnController.getLeftY()));
    SmartDashboard.putNumber("YAxis", johnController.getLeftX());
    SmartDashboard.putNumber("XAxis", johnController.getLeftX());
    SmartDashboard.putNumber("johnRightEncoder Position", m_exampleSubsystem.johnRightEncoder.getPosition());
    SmartDashboard.putNumber("johnLeftEncoder Position", m_exampleSubsystem.johnLeftEncoder.getPosition());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
