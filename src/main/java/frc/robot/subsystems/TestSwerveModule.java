// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class TestSwerveModule extends SubsystemBase {
  private CANSparkMax turnMotor;
  private CANSparkMax moveMotor;
  /** Creates a new ExampleSubsystem. */
  public TestSwerveModule(int TURN_MOTOR_ID, int MOVE_MOTOR_ID) {
    turnMotor = new CANSparkMax(TURN_MOTOR_ID, MotorType.kBrushless);
    moveMotor = new CANSparkMax(MOVE_MOTOR_ID, MotorType.kBrushless);
    //change as needed
    turnMotor.setInverted(true);
    moveMotor.setInverted(true);

    turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    moveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    //sets max current
    turnMotor.setSmartCurrentLimit(25);
    moveMotor.setSmartCurrentLimit(30);
    //limits acceleration
    turnMotor.setOpenLoopRampRate(0.4);
    moveMotor.setOpenLoopRampRate(0.4);
  }

  public void setTurnMotor(double speed) {
    turnMotor.set(speed);
  }

  public void setMoveMotor(double speed) {
    moveMotor.set(speed);
  }
}
