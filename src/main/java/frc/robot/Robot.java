/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Importer les librairies
package frc.robot;


import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.buttons.Trigger;


//Définir les composantes
public class Robot extends TimedRobot {
  private PIDController m_pidController;
  private WPI_TalonFX m_lanceur;
  private PIDController m_pidController2;
  private WPI_TalonFX m_leftMotor;
  private WPI_TalonFX m_rightMotor;
  private WPI_TalonFX m_leftMotor2;
  private WPI_TalonFX m_rightMotor2;
  private WPI_VictorSPX m_rightClimb;
  private WPI_VictorSPX m_leftClimb;
  private WPI_VictorSPX m_intakeRoller;
  private WPI_VictorSPX m_conveyorLow;
  private WPI_VictorSPX m_conveyorHigh;
  private WPI_VictorSPX m_feederBall;
  private WPI_TalonSRX m_intakeArm;
  private WPI_TalonSRX m_shooter1;
  private WPI_TalonSRX m_shooter2;
  private DigitalInput intakeArmLow;
  private DigitalInput intakeArmHigh;
  private DigitalInput rightClimbStop;
  private DigitalInput leftClimbStop;
  private DifferentialDrive m_robotDrive;
  private Joystick m_stick;
  private Compressor c;
  private WPI_TalonFX m_test;
  private AHRS ahrs;
  private static final double kP = -.075;
  private static final double kP2 = -.075;
  private static final double kI = -0.00;
  private static final double kD = -0.0;
  private static final double kD2 = -0.0;
  boolean jamaisattetint = true;
  boolean jamaisattetint2 = true;
  boolean jamaisattetint3 = true;
  boolean jamaisattetint4 = true;
  boolean jamaisattetintroule = false;
  private DigitalInput feederhigh;
  private DigitalInput feederlow;
  int etape = 0;
  boolean feeder = true;
  private static final double positionfeederouvert = 1;
  private static final double positionfeederfermer = 0.5;
  boolean shoot = true;
  int direction = m_stick.getPOV(0);
  // enum Sequences
  // {
  // AVANCER1, TOURNERGAUCHE, AVANCER2, VISER,
  // LANCER, FIN;
  // }

  // Accéder aux données du limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  @Override
  public void robotInit() {
    m_test = new WPI_TalonFX(01);
    m_leftMotor = new WPI_TalonFX(3);
    m_leftMotor2 = new WPI_TalonFX(4);
    m_rightMotor = new WPI_TalonFX(1);
    m_rightMotor2 = new WPI_TalonFX(2);
    m_rightClimb = new WPI_VictorSPX(5);
    m_leftClimb = new WPI_VictorSPX(6);
    m_intakeRoller = new WPI_VictorSPX(7);
    m_conveyorLow = new WPI_VictorSPX(8);
    m_conveyorHigh = new WPI_VictorSPX(9);
    m_feederBall = new WPI_VictorSPX(10);
    m_intakeArm = new WPI_TalonSRX(11);
    m_shooter1 = new WPI_TalonSRX(12);
    m_shooter2 = new WPI_TalonSRX(13);
    intakeArmLow = new DigitalInput(0);
    intakeArmHigh = new DigitalInput(1);
    rightClimbStop = new DigitalInput(2);
    leftClimbStop = new DigitalInput(3);
    feederhigh = new DigitalInput(4);
    feederlow = new DigitalInput(5);
   // intakeArmLow = new Button();



    m_lanceur = new WPI_TalonFX(-1);// changer

    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    m_stick = new Joystick(0);

    ahrs = new AHRS(SPI.Port.kMXP);

    m_pidController = new PIDController(kP, kI, kD, 0.02);
    m_pidController2 = new PIDController(kP2, kI, kD2, 0.02);
    m_pidController.setSetpoint(90.0);
    m_pidController2.setSetpoint(0.0);

    m_test.setSelectedSensorPosition(0);

    // Faire suivre les autres moteurs
    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);

    c = new Compressor(0);
  }

  // ------------------------------------------------------------------------
  // Méthodes

  // Allumer le compresseur
  public void allumerCompresseur() {
    c.start();
  }

  // Fermer le compresseur
  public void fermerCompresseur() {
    c.stop();
  }

  // Refroidir les moteurs si + que 40 degrés
  public void refroidirMoteurs(double m_temperature) {
    // changer pour modifier le maximum de temperature
    if (m_temperature > 40) {
      c.start();
    } else {
      c.stop();
    }
  }

  // Lancer
  public void lancer() {
    try {
      m_lanceur.set(0.7);
      TimeUnit.SECONDS.sleep(3);
    } catch (InterruptedException e) {
      e.printStackTrace();
    } finally {
      m_lanceur.set(0.0);
    }
  }

  public void suivreBalle(double x) {
    m_robotDrive.arcadeDrive(-0.7, -x * 0.04);
  }

  // ------------------------------------------------------------------------

  // Conduire avec 'arcade drive'
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(m_stick.getY(), -m_stick.getX());

    // détecter la chaleur des moteurs
    double m_temperature = m_test.getTemperature();
    refroidirMoteurs(m_temperature);
    SmartDashboard.putNumber("m_temperature", m_temperature);

    // Lire les données du limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);


    //-------------------------------------------
    //limit switches intake arm
    double a = m_stick.getRawAxis(3);
    double b;

    if (a < 0.0 & intakeArmLow.get() == true) {
    b = 0.0;
    } else {
      if (a > 0.0 & intakeArmHigh.get() == true ) {
        b = 0.0;
      } else {
        b = a;
      }
    }

    m_intakeArm.set(b);

   //-------------------------------------------- 

//-------------------------------------------
    //limit switches climb
    double c = m_stick.getRawAxis(3);
    double d;
    if (c < 0.0 & leftClimbStop.get() == true) {
    d = 0.0;
    } else {
      if (c > 0.0 & leftClimbStop.get() == true ) {
        d = 0.0;
      } else {
        d = c;
      }
    }

    m_leftClimb.set(b);

   //-------------------------------------------- 
 //limit switches climb
 double e = m_stick.getRawAxis(3);
 double f;
 if (e < 0.0 & rightClimbStop.get() == true) {
 f = 0.0;
 } else {
   if (e > 0.0 & rightClimbStop.get() == true ) {
     f = 0.0;
   } else {
     f = e;
   }
 }
 m_rightClimb.set(b);
 //----------------------------------------------

//shooter
double m_distanceshooter = m_shooter1.getSelectedSensorPosition();
if (m_stick.getRawButtonReleased(7) & shoot == true) {
 m_shooter1.set(0.4);
  m_shooter2.follow(m_shooter1);
  shoot = false;
  
   
}
if (m_stick.getRawButtonReleased(7) & shoot == false) {
  m_shooter1.set(-0.4);
   m_shooter2.follow(m_shooter1);
   shoot = true;
  
}  
SmartDashboard.putNumber("m_shooter", m_distanceshooter);
//DPAD
if (direction == 0) {
 
}
    //---------------------------------------------------
//mini servo




    //---------------------------------------------------
//feeder
     double m_distancefeeder = m_feederBall.getSelectedSensorPosition();
     if (m_stick.getRawButtonReleased(8) & feeder == true & feederhigh.get() == true & feederlow.get() == false) {
       m_feederBall.set(0.9);
       feeder = false;
      
     }
     if (m_stick.getRawButtonReleased(8) & feeder == false & feederlow.get() == true &  feederhigh.get() == false) {
       m_feederBall.set(-0.9);
       feeder = true;
     }
     SmartDashboard.putNumber("m_distancefeeder", m_distancefeeder);
    // Lire les données du navX
    double anglemesure = ahrs.getYaw();
    double vitesseangulaire = ahrs.getRawGyroX();

    double pidOut2 = m_pidController2.calculate(anglemesure);
    double pidOut = m_pidController.calculate(anglemesure);

    // Allumer les compresseur
    if (m_stick.getRawButton(5)) {
      allumerCompresseur();
    } else {
      fermerCompresseur();
    }

    // Lancer
    if (m_stick.getRawButton(6))
      lancer();

    SmartDashboard.putNumber("anglemesure", anglemesure);
    SmartDashboard.putNumber("vitesseangulaire", vitesseangulaire);
    // Poster au smart dashboard les données du limelight
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    SmartDashboard.putNumber("pidOut2", pidOut2);
    SmartDashboard.putNumber("pidOut", pidOut);

    // Système de controle automatique
    if (m_stick.getRawButton(1))
      suivreBalle(x);

    // Tourner a un angle
    if (m_stick.getRawButton(2))
      ahrs.reset();
    // faire
    // m_pidController2.setSetpoint(0.0);

    if (m_stick.getRawButton(3)) {
      // double erreur = 90.0 - anglemesure;
      m_robotDrive.arcadeDrive(0.0, pidOut);
    }

    if (m_stick.getRawButton(4)) {
      m_robotDrive.arcadeDrive(-0.7, pidOut2);
    }
    if (m_stick.getRawButton(7)) {
      m_test.setSelectedSensorPosition(0);

    }
    double m_distance = m_test.getSelectedSensorPosition();
    SmartDashboard.putNumber("m_distance", m_distance);

  }// Fin du teleop.periodic

  public void autonomousPeriodic() {

    // partie autonome
    double m_distance = m_test.getSelectedSensorPosition();
    SmartDashboard.putNumber("m_distance", m_distance);
    double anglemesure = ahrs.getYaw();
    int index = 1;

    // Sequences TOURNERGAUCHE;

    switch (etape) {

    case 0:

      if (jamaisattetint) {
        if (m_distance < 5000) {
          m_test.set(0.07);
        } else {
          m_test.set(0.0);
          jamaisattetint = false;

          etape = index++;
        }
      }

      break;
    case 1:
      // Tourner a gauche 90 degrés
      ahrs = new AHRS(SPI.Port.kMXP);
      ahrs.reset();
      m_pidController = new PIDController(kP, kI, kD, 0.02);
      m_pidController.setSetpoint(90.0);

      double pidOut = m_pidController.calculate(anglemesure);
      m_robotDrive.arcadeDrive(0.0, pidOut);
      etape = index++;
      break;

    case 2:
      m_test.setSelectedSensorPosition(0);
      if (jamaisattetint2) {
        if (m_distance < 5000) {
          m_test.set(0.07);

        } else {
          m_test.set(0.0);
          jamaisattetint2 = false;
          etape = index++;
        }
      }
      break;

    case 3:
      ahrs = new AHRS(SPI.Port.kMXP);
      ahrs.reset();
      m_pidController = new PIDController(kP, kI, kD, 0.02);
      m_pidController.setSetpoint(90.0);
      m_pidController.calculate(anglemesure);
      m_robotDrive.arcadeDrive(0.0, 0.0);
      etape = index++;
      break;

    case 4:
      m_test.setSelectedSensorPosition(0);
      if (jamaisattetint4) {
        if (m_distance < 5000) {
          m_test.set(0.07);
        } else {
          m_test.set(0.0);
          jamaisattetint4 = false;
        }
      }
      break;

    }
  }
}
