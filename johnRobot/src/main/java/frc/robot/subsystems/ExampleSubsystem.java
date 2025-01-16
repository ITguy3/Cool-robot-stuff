// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final SparkMax johnRightMotor = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax johnLeftMotor = new SparkMax(2, MotorType.kBrushless);

  private final SparkMaxConfig johnSMConfig = new SparkMaxConfig();

  DifferentialDrive johnDDrive = new DifferentialDrive(johnLeftMotor, johnRightMotor);
  public ExampleSubsystem() {
    johnSMConfig.inverted(true);
    johnRightMotor.configure(johnSMConfig, null, null);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

boolean forward = false;
boolean backward = false;
boolean right = false;
boolean left = false;

  @Override
  public void periodic() {
    /* 
    if (backward == true) {
      forward = false;
      johnDDrive.arcadeDrive(-0.25, 0);
    }

    if (right == true) {
      left = false;
      johnDDrive.arcadeDrive(0.1, 10);
    }

    if (left == true) {
      right = false;
      johnDDrive.arcadeDrive(0.1, -10);
    }

    */
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  public Command johnMoveForward(DoubleSupplier xAxis, DoubleSupplier yAxis) {
  
    return new InstantCommand(()-> {
      johnDDrive.arcadeDrive(0.25*(yAxis.getAsDouble()), 0);
    });
  }

  public Command johnMoveBackward() {
    return new InstantCommand(()-> {
      backward = true;
    });
  }

  public Command johnTurnRight() {
    return new InstantCommand(()-> {
      johnDDrive.arcadeDrive(0.25, -0.4);
    });
  }

  public Command johnTurnLeft() {
    return new InstantCommand(()-> {
      johnDDrive.arcadeDrive(0.25, 0.4);
    });
  }

  public Command johnStop() {
    return new InstantCommand(()-> {
      //johnDDrive.arcadeDrive(0, 0);
      forward = false;
      backward = false;
    });
  }

  /*
  public Command johnVoltage(double voltage) {
    return new InstantCommand(()-> {
      johnLeftMotor.setVoltage(0.12 + voltage);
      johnRightMotor.setVoltage(-0.11 - voltage);
    });
  }
  
  
  double johnLeftVoltage = 0.0;
  public Command johnIncrement() {
    return new InstantCommand(()-> {
      johnLeftVoltage+=0.01;
      johnDDrive.tankDrive(0, johnLeftVoltage); //left voltage = 0.12, right voltage = 0.11
      SmartDashboard.putNumber("Voltage: ", johnLeftVoltage);
    });
  }
    */
}
