package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {


    //Motor ID's
    public static final double algaewristmotorID = 17;
    public static final double algaerollermotorID = 18;
    public static final double coralwristmotorID = 15;
    public static final double coralrollermotorID = 16;
    public static final double elevatormotorID = 14;

    //coral variables
    public double coralposerror;
    public static final double coralkP = 0.04;
    public static final double coraltargetpos = 7;
    public static final double coralhumanintakepos = 18.5;
    public static final double coraloutpos = 9;
    public double coralwristpower;
    
    //algae variables
    public double algaeposerror;
    public double algaepower;
    public double algaekP = 0.08;
    public double algaetargetpos = 9;
    public double downpos = 9;
    public double upposition = 3;
    public int choosetargetpos = 0;

    //elevator variables
    public double elevatorposerror;
    public double elevatortargetpos = 0;
    public double elevatorpower;
    public double elevatorkG = 0.0355;
    public int choosethingie = 0;
    public double l1 = 0;
    public double l2 = 16;
    public double l3 = 39;
    public double lift = 49;

    //controllers
    public XboxController Operator = new XboxController(1);

}
