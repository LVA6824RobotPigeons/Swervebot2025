// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.pheonix6.hardware.TalonFXS;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */

  private TalonFXS Elevatormotor = new TalonFXS(14,"rio");

  public ElevatorSubsystem() {}

  public Command SendElevatorToTargetPos() {
    if(choosethingie == 0){
      elevatortargetpos = l1;
    } else if(choosethingie == 1){
      elevatortargetpos = l2;
    } else if(choosethingie == 2){
      elevatortargetpos = l3;
    }

    elevatorposerror = elevatortargetpos-elevatorpos;

    if(elevatorposerror > 0){
      elevatorpower = 0.45;
    }else if(elevatorposerror < 0 && elevatorposerror > -2){
      elevatorpower = elevatorkG;
    }else {
      elevatorpower = -0.3;
    }

    Elevatormotor.set(elevatorpower);
  }

  ElevatorSubsystem.setDefaultCommand(SendElevatorToTargetPos);

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var elevaorrotorPosSignal = Elevatormotor.getPosition();
    var elevatorpos = elevaorrotorPosSignal.getValueAsDouble;

  }
}
