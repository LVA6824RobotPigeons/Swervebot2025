// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.XboxController;
//import frc.robot.subsystems.PWMTalonFXS;
//import frc.robot.subsystems.PWMTalonFXS.MotorArrangement;

import com.ctre.phoenix6.hardware.TalonFXS;
//import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
//import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //algae wrist motor
  private TalonFXS algaeWrist = new TalonFXS(17, "rio");
  //algae roller
  private TalonFXS algaeRoller = new TalonFXS(18, "rio");
  //coral wrist motor
  private TalonFXS coralWrist = new TalonFXS(15,"rio");
  //coral roller motor
  private TalonFXS coralRoller = new TalonFXS(16,"rio");

  //coral variables
  double coralPoserror; //error between the current position and the target position found by subtracting current position from target position
  double coralkP = 0.08; //value that the error is multiplied by to get proportional input into the coral wrist
  double coraltargetpos = 3; //set the default target position for the coral wrist
  double coralwristpower;
  double coralhumanpos = 10; //target position for picking up coral from human player -still needs alot of tweaking
  double coraloutpos = 3; //target position for putting coral in the reed

  //creates new talon FXS on can id 18 which is our algae roller
  //private PWMTalonFXS algaeRoller = new PWMTalonFXS(18);

  //controller for robot operator
  private XboxController Operator = new XboxController(1);


  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

    //algaeRoller.setMotorArrangement(MotorArrangement.NEO550_JST);

    //algaeRoller.setNeutralMode(false);

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
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    coralWrist.setPosition(0);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {

    //get raw encoder data
    var coralrotorPosSignal = coralWrist.getPosition();
    var coralrotorPos = coralrotorPosSignal.getValueAsDouble();

    coralPoserror = coraltargetpos-coralrotorPos;
    
    //code to make sure coral wrist power is never negative ie. bang bang ctrl
    //if(coralPoserror > 0) {
    //  coralwristpower = coralPoserror*coralkP;
    //}else {
    //  coralwristpower = 0;
    //}

    //lets coral wrist power go into negative to make the wrist ehav more like a pid system except only with proportional control
    coralwristpower = coralPoserror*coralkP;

    

    coralWrist.set(coralwristpower);

    //print the encoder value - for debugging remove later
    System.out.println(coralrotorPos);
    //print the calculated coral wrist power - for debugging remove later
    //System.out.println(coralwristpower);

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
      algaeRoller.set(0.2);
    } else if(Operator.getBButton()==true) {
      algaeRoller.set(-0.2);
    } else {
      algaeRoller.set(0);
    }


    //controls coral rollers
    if(Operator.getXButton()==true){
      coralRoller.set(0.5);
    } else if(Operator.getYButton()==true) {
      coralRoller.set(-0.5);
    } else {
      coralRoller.set(0);
    }


    //update coral encoder position
    coralrotorPosSignal.waitForUpdate(0.050);


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
