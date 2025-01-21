// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;

public class TankDrive extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final SparkMax johnRightMotor = new SparkMax(3, MotorType.kBrushless);
  public final SparkMax johnLeftMotor = new SparkMax(2, MotorType.kBrushless);

  public final SparkRelativeEncoder johnRightEncoder = (SparkRelativeEncoder) johnRightMotor.getEncoder();
  public final SparkRelativeEncoder johnLeftEncoder = (SparkRelativeEncoder) johnLeftMotor.getEncoder();

  private final SparkMaxConfig johnSMConfig = new SparkMaxConfig(); // configuration for the right motor

  private final PIDController johnPIDController = new PIDController(0.034472, 0, 0); // ADD PROPER VALUES!!!!!!

  //DifferentialDrive johnDDrive = new DifferentialDrive(johnLeftMotor, johnRightMotor);

  private Voltage johnVoltage = Volts.of(0);

  public TankDrive() {
    johnSMConfig.idleMode(IdleMode.kBrake); // makes motors brake as soon as it gets unpowered
    johnLeftMotor.configure(johnSMConfig, null, null);
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


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Encoder Value",johnRightEncoder.getPosition());
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("At setpoint?", johnPIDController.atSetpoint());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command johnMove(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    Command cmd =  new InstantCommand(()-> {
      //johnDDrive.arcadeDrive(-0.5 * yAxis.getAsDouble(), -0.4 * xAxis.getAsDouble());
    });

    cmd.addRequirements(this);
    

    return cmd;
  }

  public Command johnMoveSetDistance(double desiredDistance    /* in feet */) {

    Command cmd = new InstantCommand(() -> {
      johnPIDController.setSetpoint(johnLeftEncoder.getPosition() + desiredDistance);
      johnPIDController.setTolerance(0.25);
    }).andThen(new RepeatCommand(Commands.runOnce(() -> {
      double h = johnPIDController.calculate(johnLeftEncoder.getPosition());

      johnLeftMotor.setVoltage(h);
      johnRightMotor.setVoltage(h);

      SmartDashboard.putNumber("Voltage", h);

    })).until(()->johnPIDController.atSetpoint()));

    cmd.addRequirements(this);

    return cmd;
  }

  public Command johnRunSYSID(SparkMax motor, String name) {
    SysIdRoutine jon = new SysIdRoutine(new SysIdRoutine.Config(
      Volts.per(Second).ofBaseUnits(1.5),
      Volts.of(2),
      Seconds.of(5)
    ), new SysIdRoutine.Mechanism(
      drive -> {
        motor.setVoltage(drive.magnitude());
        johnVoltage = drive;
      },
      log -> {
        log.motor(name)
          .voltage(johnVoltage)
          .angularPosition(Rotations.of(motor.getEncoder().getPosition()))
          .angularVelocity(RotationsPerSecond.of(motor.getEncoder().getVelocity() / 60));

          SmartDashboard.putNumber("logged position", motor.getEncoder().getPosition());
          SmartDashboard.putNumber("logged velocity", motor.getEncoder().getVelocity() / 60);
      }, this

    ));

    return jon.quasistatic(SysIdRoutine.Direction.kForward)
    .andThen(new WaitCommand(1.0))
    .andThen(jon.dynamic(SysIdRoutine.Direction.kForward)
    .andThen(new WaitCommand(1.0))
    .andThen(jon.quasistatic(SysIdRoutine.Direction.kReverse))
    .andThen(new WaitCommand(1.0))
    .andThen(jon.dynamic(SysIdRoutine.Direction.kReverse)));
  }

/*
  public Command johnMoveForward(DoubleSupplier xAxis, DoubleSupplier yAxis) {
  
    return new InstantCommand(()-> {
      johnDDrive.arcadeDrive(-0.25*(yAxis.getAsDouble()), 0);
    });
  }

  public Command johnMoveBackward(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    return new InstantCommand(()-> {
      johnDDrive.arcadeDrive(0.25*(yAxis.getAsDouble()), 0);
    });
  }

  public Command johnTurnRight(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    return new InstantCommand(()-> {
      johnDDrive.arcadeDrive(0.25, 0.4 * xAxis.getAsDouble());
    });
  }

  public Command johnTurnLeft(DoubleSupplier xAxis, DoubleSupplier yAxis) {
    return new InstantCommand(()-> {
      johnDDrive.arcadeDrive(0.25, -0.4 * xAxis.getAsDouble());
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
