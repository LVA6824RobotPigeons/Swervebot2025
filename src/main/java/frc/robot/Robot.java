// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;

import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.commands.*;


import com.ctre.phoenix6.hardware.TalonFXS;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command testcommand;

  //algae wrist motor
  private TalonFXS algaeWrist = new TalonFXS(17, "rio");
  //algae roller
  private TalonFXS algaeRoller = new TalonFXS(18, "rio");
  //coral wrist motor
  private TalonFXS coralWrist = new TalonFXS(15,"rio");
  //coral roller motor
  private TalonFXS coralRoller = new TalonFXS(16,"rio");
  //elevator motor
  private TalonFXS elevatorMotor = new TalonFXS(14,"rio");

  //coral variables
  public double coralPoserror; //error between the current position and the target position found by subtracting current position from target position
  public double coralkP = 0.04; //value that the error is multiplied by to get proportional input into the coral wrist
  public double coraltargetpos = 7; //set the default target position for the coral wrist
  public double coralwristpower;
  double coralhumanpos = 18.5; //target position for picking up coral from human player -still needs alot of tweaking
  public double coraloutpos = 9; //target position for putting coral in the reef

  //elevator variables and stuffs
  double elevatorPoserror;
  double elevatortargetpos = 0; //default position at start of the tele-op
  double elevatorpower;
  double elevatorkG = 0.0355;
  int choosethingie = 0;
  double l1 = 0;
  double l2 = 16;
  double l3 = 39;
  double lift = 49; 

  //algae variables
  double algaePoserror;
  double algaetargetpos = 9;
  double algaepower;
  double algaekP  = 0.08;
  int choosetargetpos = 0;
  double downpos = 9; // position of elevator when down to be able to intake algae
  double uppostition = 3; // set this higher to reduce probability of algae intake getting hit during match but dont set to high or else elevator cant move

  

  //controller for robot operator
  private XboxController Operator = new XboxController(1);

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    NamedCommands.registerCommand("testcommand", testcommand);
    
    CameraServer.startAutomaticCapture();

    SmartDashboard.putNumber("Elevator POS:", choosethingie);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    //if (testcommand != null) {
    //  System.out.println("test worked");
    //}
  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    coralWrist.setPosition(0);
    elevatorMotor.setPosition(0);
    elevatortargetpos = l1;
    choosethingie = 0;
    algaeWrist.setPosition(0);
    choosetargetpos = 0;
    algaetargetpos = downpos;

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    //get raw encoder data for coral wrist
    var coralrotorPosSignal = coralWrist.getPosition();
    var coralrotorPos = coralrotorPosSignal.getValueAsDouble();

    //get raw encoder data for elevator
    var elevatorrotorPosSignal = elevatorMotor.getPosition();
    var elevatorPos = elevatorrotorPosSignal.getValueAsDouble();

    //get raw ecoder data for algae
    var algaerotorPosSignal = algaeWrist.getPosition();
    var algaerotorPos = algaerotorPosSignal.getValueAsDouble();

    if(Operator.getBackButton()==true) {
      algaeWrist.setPosition(9.5);
    }

    //takes in trigger input and changes variable to act as a toggle
    if(Operator.getRightTriggerAxis() > 0.1 && choosethingie < 2){
      choosethingie = choosethingie+1;
      try {
        Thread.sleep(250);
      } catch(InterruptedException e) {
      }
    }
    if(Operator.getLeftTriggerAxis() > 0.1 && choosethingie > 0){
      choosethingie = choosethingie-1;
      try {
        Thread.sleep(250);
      } catch(InterruptedException e) {
      }
    }
    if(Operator.getStartButton()==true) {
      choosethingie = 3;
    }

    //takes in joystick input and changes a variable to act as a toggle
    if(Operator.getLeftY() < -0.1 && choosetargetpos < 1){
      choosetargetpos = choosetargetpos+1;
      try {
        Thread.sleep(250);
      } catch(InterruptedException e) {
      }
    }

    if(Operator.getLeftY() > 0.1 && choosetargetpos > 0){
      choosetargetpos = choosetargetpos-1;
      try {
        Thread.sleep(250);
      } catch(InterruptedException e) {
      }
    }

    //sets the target position of the algae intake based on joystick input
    if(choosetargetpos == 0){
      algaetargetpos = downpos;
    } else if(choosetargetpos == 1){
      algaetargetpos = uppostition;
    }

    //sets the target position of the elevator based on trigger input
    if(choosethingie == 0){
      elevatortargetpos = l1;
    } else if(choosethingie == 1) {
      elevatortargetpos = l2;
    } else if(choosethingie == 2) {
      elevatortargetpos = l3;
    } else if(choosethingie == 3) {
      elevatortargetpos = lift;
    }

    //coral position error
    coralPoserror = coraltargetpos-coralrotorPos;

    //elevator position error
    elevatorPoserror = elevatortargetpos-elevatorPos;

    //algae position error
    algaePoserror = algaetargetpos-algaerotorPos;

    //decides how much power to give motor based on error and kp
    if(algaePoserror > 0) {
      algaepower = algaePoserror*algaekP;
    }else if(algaePoserror < 0 && algaePoserror > -2) {
      algaepower = 0;
    }else {
      algaepower = -0.1;
    }
    
    //code to make sure coral wrist power is never negative ie. bang bang ctrl
    if(coralPoserror > 0) {
      coralwristpower = coralPoserror*coralkP;
    }else if(coralPoserror < 0 && coralPoserror > -1) {
      coralwristpower = 0;
    }else {
      coralwristpower = -0.1;
    }

    //takes error and multiplies it by kP to get elevator power but only if the error is a positive number
    if(elevatorPoserror > 0){
      elevatorpower = 0.45;
    }else if(elevatorPoserror < 0 && elevatorPoserror > -2){ //
      elevatorpower = elevatorkG;
    }else {
      elevatorpower = -0.3;
    }

    //prints out the amount of power given to the algae motor
    //System.out.println(algaepower);
    algaeWrist.set(algaepower);

    //gives power to coral wrist motor
    coralWrist.set(coralwristpower);

    //gives power to elevator motor
    elevatorMotor.set(elevatorpower); //do not touch!!! -sets the elevator motor power to the correct power
    //System.out.println(elevatorPos);

    //print the encoder value - for debugging remove later
    //System.out.println(algaerotorPos);
    //print the calculated coral wrist power - for debugging remove later
    //System.out.println(coralwristpower);
    //print elevator encoder value - for debugging remove later
    //System.out.println(elevatorPoserror); //print out the choose thingie variable

    //makes the algae intake move to the human player intake position when i press the right bumper
    if(Operator.getRightBumperButton()==true){
      coraltargetpos=coralhumanpos;
    }
    //makes the algae intake move to the reef output position when i press left trigger
    if(Operator.getLeftBumperButton()==true){
      coraltargetpos=coraloutpos;
    }

    //controls algae rollers
    if(Operator.getAButton()==true){
      algaeRoller.set(0.9);
    } else if(Operator.getBButton()==true) {
      algaeRoller.set(-0.9);
    } else {
      algaeRoller.set(-0.03);
    }


    //controls coral rollers
    if(Operator.getXButton()==true){
      coralRoller.set(0.4);
    } else if(Operator.getYButton()==true) {
      coralRoller.set(-0.4);
    } else {
      coralRoller.set(0);
    }

    if(Operator.getYButtonReleased()==true){
      coralRoller.clearStickyFault_DeviceTemp();
    }
    //update coral encoder position
    coralrotorPosSignal.waitForUpdate(0.05);
    //update elevator encoder position
    elevatorrotorPosSignal.waitForUpdate(0.05);

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
