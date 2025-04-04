// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFXS;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class publishCommands extends Command {
    /** Creates a new publishCommands. */
    //public publishCommands(Robot robot) {
    //TalonFXS coralWrist = new TalonFXS(15,"rio");
    //NamedCommands.registerCommand("L1CoralOut", new SequentialCommandGroup());
    //}

  public void hold_pos(Robot robot) {

    //var coralrotorPosSignal = coralWrist.getPosition();
    //var coralrotorPos = coralrotorPosSignal.getValueAsDouble();


    //robot.coralPoserror = robot.coraltargetpos-robot.coralrotorPos;

    //if(robot.coralPoserror > 0) {
    //  robot.coralwristpower = robot.coralPoserror*robot.coralkP;
    //}else if(robot.coralPoserror < 0 && robot.coralPoserror > -1) {
    //  robot.coralwristpower = 0;
    //}else {
    //  robot.coralwristpower = -0.1;
    //}
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
