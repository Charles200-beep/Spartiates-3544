/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Importer les librairies
package frc.robot;

import java.util.concurrent.TimeUnit;

//import java.util.concurrent.TimeUnit;
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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

// Définir les sequences
enum SequencesAutonome {
  AVANCER_1, TOURNER_1, TOURNER_2, AVANCER_2, VISER, LANCER, FIN, AVANCER_3;
}

enum SequencesLancer {
  ACCELERER, OUVRIRPORTE, FERMERPORTE, VISER, ALIMENTER, FIN;
}

enum SequencesIntakeArm {
  ACCELERER, BAISSER, MONTER, FIN;
}

// Définir les composantes
public class Robot extends TimedRobot {
  private PIDController m_pidController;
  // private WPI_TalonFX m_lanceur;
  // private PIDController m_pidController2;
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
  // private FakeMotor m_test;
  private AHRS ahrs;
  private Timer shooterSpoolUpTimer;
  private Timer unlockTimer;

  // private FakeAHRS fakeahrs;
  // private static final double ratioGearboxRoues = 8.68;
  // private static final double diametreRoues = 15.24; // en centimetres
  // private static final double rotationNombre = 2048;

  // Valeurs pour vitesses de moteurs
  private static final double vitesseConvoyeur = 0.7;
  private static final double vitesseLanceur = 1.0;
  private static final double vitesseGrimpeur = 0.7;
  private static final double vitesseIntakeArm = 0.7;
  private static final double vitesseFeeder = 0.7;
  private static final double vitesseSuivreBalle = 0.7;

  // Boutons et axes de joystick XBOX Conducteur
  private static final int axeConduireAvant = 0;
  private static final int axeConduireTournerDroit = 5;
  private static final int axeConduireTournerGauche = 4;
  private static final int axeGrimpeurDroit = 0;
  private static final int axeGrimpeurGauche = 0;
  private static final int axeIntakeArm = 3;
  private static final int boutonIntakeArmRaise = 0;
  private static final int boutonIntakeArmlower = 0;
  private static final int boutonLanceurToggle = 0;
  private static final int boutonConvoyeurIn = 0;
  private static final int boutonConvoyeurOut = 0;
  private static final int boutonFeeder = 0;

  // Boutons et axes de joystick Pilote 2
  private static final int boutonGrimpGaucheUp = 6;
  private static final int boutonGrimpGaucheDown = 7;
  private static final int boutonGrimpDroiteUp = 10;
  private static final int boutonGrimpDroiteDown = 11;
  private static final int boutonIntakeRoller = 3;

  // Gains
  private static final double kP_SuivreBalle = 0.04;
  private static final double kP = -.075;
  private static final double kI = -0.00;
  private static final double kD = -0.0;

  // Servo Motor Positions
  private static final double rightClimbRatchetLocked = 0.0;
  private static final double rightClimbRatchetUnLocked = 90.0;
  private static final double leftClimbRatchetLocked = 180.0;
  private static final double leftClimbRatchetUnLocked = 90.0;

  // Initialisation automatique
  boolean initialiser = true;
  double distance = 0.0;
  SequencesAutonome step = SequencesAutonome.AVANCER_1;
  SequencesAutonome step2 = SequencesAutonome.AVANCER_1;
  SequencesAutonome step3 = SequencesAutonome.AVANCER_1;
  double m_distance;
  double anglemesure;
  double distanceautonome1 = 1128726;
  double distanceautonome2 = 519808;
  double distanceautonome3 = 853970;

  // private static final double kD2 = -0.0;
  // boolean jamaisattetint = true;
  // boolean jamaisattetint2 = true;
  // boolean jamaisattetint3 = true;
  // boolean jamaisattetint4 = true;
  // boolean jamaisattetintroule = false;
  private DigitalInput feederhigh;
  private DigitalInput feederlow;
  int etape = 0;
  boolean feeder = true;
  boolean conveyor1 = true;
  boolean shoot = false;
  boolean conveyor = false;
  boolean dejaReset = false;
  boolean dejaReset2 = false;
  // int direction = m_stick.getPOV(0);
  private Servo m_leftClimbRatchet;
  private Servo m_rightClimbRatchet;
  double angle = 0.0;
  private Joystick m_stick2;

  // Accéder aux données du limelight
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  @Override
  public void robotInit() {
    // m_test = new FakeMotor(01);
    m_leftMotor = new WPI_TalonFX(3);
    m_leftMotor2 = new WPI_TalonFX(4);
    m_rightMotor = new WPI_TalonFX(1);
    m_rightMotor2 = new WPI_TalonFX(2);
    m_rightClimb = new WPI_VictorSPX(5);
    m_leftClimb = new WPI_VictorSPX(10);
    m_intakeRoller = new WPI_VictorSPX(7);
    m_conveyorLow = new WPI_VictorSPX(8);
    m_conveyorHigh = new WPI_VictorSPX(9);
    // m_feederBall = new WPI_VictorSPX(10);
    m_intakeArm = new WPI_TalonSRX(11);
    m_shooter1 = new WPI_TalonSRX(12);
    m_shooter2 = new WPI_TalonSRX(13);
    intakeArmLow = new DigitalInput(0);
    intakeArmHigh = new DigitalInput(1);
    rightClimbStop = new DigitalInput(2);
    leftClimbStop = new DigitalInput(3);
    feederhigh = new DigitalInput(4);
    feederlow = new DigitalInput(5);
    m_intakeRoller = new WPI_VictorSPX(7);
    m_rightClimbRatchet = new Servo(0);
    m_leftClimbRatchet = new Servo(1);
    m_stick2 = new Joystick(1);
    shooterSpoolUpTimer = new Timer();
    // m_lanceur = new WPI_TalonFX(-1);// changer
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_stick = new Joystick(0);
    ahrs = new AHRS(SPI.Port.kMXP);
    m_pidController = new PIDController(kP, kI, kD, 0.02);
    // m_pidController2 = new PIDController(kP2, kI, kD2, 0.02);
    m_pidController.setSetpoint(90.0);
    // m_pidController2.setSetpoint(0.0);
    // m_test.setSelectedSensorPosition(0);
    c = new Compressor(0);
    unlockTimer = new Timer();
    // Faire suivre les autres moteurs
    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);
    m_shooter2.follow(m_shooter1);
    m_conveyorHigh.follow(m_conveyorLow);
    shooterSpoolUpTimer.start();
    unlockTimer.start();
    m_rightClimb.setInverted(true);
    m_conveyorHigh.setInverted(true);
    // TalonFX Configuration
    m_leftMotor.configOpenloopRamp(0.5);
    m_rightMotor.configOpenloopRamp(0.5);
    m_leftMotor2.configOpenloopRamp(0.5);
    m_rightMotor2.configOpenloopRamp(0.5);

    // Initialiser Servo Ratchets Grimpeur
    m_rightClimbRatchet.setAngle(rightClimbRatchetLocked);
    m_leftClimbRatchet.setAngle(leftClimbRatchetLocked);

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

  // // Lancer
  // public void lancer() {
  // try {
  // m_lanceur.set(0.7);
  // TimeUnit.SECONDS.sleep(3);
  // } catch (InterruptedException e) {
  // e.printStackTrace();
  // } finally {
  // m_lanceur.set(0.0);
  // }
  // }

  public void suivreBalle(double x) {
    m_robotDrive.arcadeDrive(-vitesseSuivreBalle, -x * kP_SuivreBalle);
  }

  public void centrerCible(double x) {
    m_robotDrive.arcadeDrive(0.0, -x * kP_SuivreBalle);
  }

  // ------------------------------------------------------------------------
  // Conduire avec 'arcade drive'
  @Override
  public void teleopPeriodic() {
    double leftTrigger = m_stick.getRawAxis(2);
    double rightTrigger = m_stick.getRawAxis(3);
    double rotation = rightTrigger - leftTrigger;
    m_robotDrive.arcadeDrive(-m_stick.getY(), rotation);

    // if (leftTrigger > -0.9 ) {
    // m_robotDrive.arcadeDrive(0, -leftTrigger);
    // }

    // if (rightTrigger > -0.9 ) {
    // m_robotDrive.arcadeDrive(0, -rightTrigger);
    // }

    // -------------------------------------------
    // détecter la chaleur des moteurs
    double m_temperature = m_leftMotor.getTemperature();
    refroidirMoteurs(m_temperature);
    SmartDashboard.putNumber("m_temperature", m_temperature);

    // Lire les données du limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // -------------------------------------------
    // // limit switches intake arm

    // double a = m_stick.getRawAxis(3);
    // double b;

    // if (a < 0.0 & intakeArmLow.get() == true) {
    // b = 0.0;
    // } else {
    // if (a > 0.0 & intakeArmHigh.get() == true) {
    // b = 0.0;
    // } else {
    // b = a;
    // }
    // }

    // m_intakeArm.set(b);

    // --------------------------------------------
    // Intake arm
    double Z = m_stick2.getRawAxis(2);
    double intakeArmPos = m_intakeArm.getSelectedSensorPosition();
    if (Z == 1) {
      m_intakeArm.set(0.2);
      // if (intakeArmPos > ) {

      // }
    }

    if (Z == -1) {
      m_intakeArm.set(-0.2);
    }

    SmartDashboard.putNumber("Z", intakeArmPos);
    // Intake Roller
    if (m_stick2.getRawButton(3)) {
      m_intakeRoller.set(-0.7);
    } else {
      m_intakeRoller.set(0.0);
    }

    // -------------------------------------------
    // limit switches climb
    // double z = m_stick.getRawAxis(3);
    // double d;
    // if (z < 0.0 & leftClimbStop.get() == true) {
    // d = 0.0;
    // } else {
    // if (z > 0.0 & leftClimbStop.get() == true) {
    // d = 0.0;
    // } else {
    // d = z;
    // }
    // }

    // m_leftClimb.set(d);

    // --------------------------------------------
    // // limit switches climb
    // double e = m_stick.getRawAxis(3);
    // double f;
    // if (e < 0.0 & rightClimbStop.get() == true) {
    // f = 0.0;
    // } else {
    // if (e > 0.0 & rightClimbStop.get() == true) {
    // f = 0.0;
    // } else {
    // f = e;
    // }
    // }
    // m_rightClimb.set(f);
    // ----------------------------------------------
    if (m_stick2.getRawButtonPressed(1)) {
      shoot = !shoot;
      shooterSpoolUpTimer.reset();
    }

    // shooter
    if (shoot) {
      m_shooter1.set(vitesseLanceur);
      m_shooter2.follow(m_shooter1);

      if (shooterSpoolUpTimer.get() > 6) {
        // shooterSpoolUpTimer.stop();
        m_conveyorHigh.set(vitesseConvoyeur);
        m_conveyorLow.follow(m_conveyorHigh);
      }

    }
    // System.out.println(shoot);
    // System.out.println("convoyeur:");
    // System.out.println(m_conveyorHigh.get());
    // System.out.println("shooter:");
    // System.out.println(m_shooter1.get());

    if (!shoot) {
      m_conveyorHigh.set(0.0);
      m_conveyorLow.follow(m_conveyorHigh);
      m_shooter1.set(0.0);
      m_shooter2.follow(m_shooter1);

    }
    // DPAD
    // if (direction == 0) {

    // }
    // ---------------------------------------------------
    // mini servo

    // SmartDashboard.putNumber("angle", angle);
    // m_leftClimbRatchet.setAngle(angle);
    // if (m_stick.getRawButtonReleased(null)) {
    // angle = angle+10;
    // }
    // if (m_stick.getRawButtonReleased(null)) {
    // angle = angle-10;

    // }
    // if (m_stick.getRawButtonReleased(null)) {
    // m_leftClimbRatchet.setAngle(0);

    // if (m_stick.getRawButtonReleased(null)) {
    // m_leftClimbRatchet.setAngle(90);

    // }

    // }
    // m_rightClimbRatchet.setAngle(angle);
    // if (m_stick.getRawButtonReleased(null)) {
    // angle = angle+10;
    // }
    // if (m_stick.getRawButtonReleased(null)) {
    // angle = angle-10;

    // }
    // if (m_stick.getRawButtonReleased(null)) {
    // m_rightClimbRatchet.setAngle(0);

    // if (m_stick.getRawButtonReleased(null)) {
    // m_rightClimbRatchet.setAngle(90);

    // }

    // }

    // ----------------------------------------------------
    // conveyor (feeder)
    m_conveyorLow.follow(m_conveyorHigh);

    if (m_stick2.getRawButton(2)) {
      m_conveyorHigh.set(vitesseConvoyeur);
    }

    if (m_stick2.getRawButton(5)) {
      m_conveyorHigh.set(-vitesseConvoyeur);
    }
    // -----------------------------------------------------
    // unlock ratchets
    if (m_stick2.getRawButton(8)) {
      m_leftClimbRatchet.setAngle(leftClimbRatchetUnLocked);

    }
    if (m_stick2.getRawButton(8)) {
      m_rightClimbRatchet.setAngle(rightClimbRatchetUnLocked);

    }
      // unlock ratchets
      if (m_stick2.getRawButton(9)) {
        m_leftClimbRatchet.setAngle(leftClimbRatchetLocked);
  
      }
      if (m_stick2.getRawButton(9)) {
        m_rightClimbRatchet.setAngle(rightClimbRatchetLocked);
  
      }
    //monter gauche
    if (m_stick2.getRawButton(6)) {
      m_leftClimb.set(-1);
  
    }
    if (m_stick2.getRawButtonReleased(6))
    m_leftClimb.set(0);
     // Baisser gauche
     if (m_stick2.getRawButton(7)) {
      m_leftClimb.set(1);
    }
    if (m_stick2.getRawButtonReleased(7))
    m_leftClimb.set(0);
  
    // Monter droite
    if (m_stick2.getRawButton(11)) {
      m_rightClimb.set(-1);
    } 
    if (m_stick2.getRawButtonReleased(11))
    m_rightClimb.set(0);
    // Baisser droite
    if (m_stick2.getRawButton(10)) {
      m_rightClimb.set(1);
    
    }
    if (m_stick2.getRawButtonReleased(10))
    m_rightClimb.set(0);

    // Lire et publier la vitesse des shooters
    double vitesseShooters = m_shooter1.get();
    SmartDashboard.putNumber("vitesse Shooters", vitesseShooters);

    // Lire et publier la position de intake arm
    double positionIntakeArm = m_intakeArm.getSelectedSensorPosition();
    SmartDashboard.putNumber("position Intake Arm", positionIntakeArm);

    // Lire les données du navX
    double anglemesure = ahrs.getYaw();

    // double pidOut2 = m_pidController2.calculate(anglemesure);
    double pidOut = m_pidController.calculate(anglemesure);

    // Allumer les compresseur
    if (m_stick.getRawButton(10)) {
      allumerCompresseur();
    } else {
      fermerCompresseur();
    }

    // Lancer
    // if (m_stick.getRawButton(6))
    // lancer();

    // Poster au smart dashboard les données du limelight
    // SmartDashboard.putNumber("Limelightx", x);
    // SmartDashboard.putNumber("LimelightArea", area);

    // SmartDashboard.putNumber("pidOut2", pidOut2);

    // Système de controle automatique
    if (m_stick.getRawButton(1))
      suivreBalle(x);

    // Se centrer
    if (m_stick.getRawButton(2))
      centrerCible(x);

    // Tourner a un angle
    if (m_stick.getRawButton(7) & m_stick.getRawButton(8))
      ahrs.reset();
    // faire
    // m_pidController2.setSetpoint(0.0);

    if (m_stick.getRawButton(5) | m_stick.getRawButton(6)) {
      m_robotDrive.arcadeDrive(0.0, pidOut);
    } 

                    // if (m_stick.getRawButton(4)) {
                    // m_robotDrive.arcadeDrive(-0.7, pidOut2);

                    // if (m_stick.getRawButton(7) & m_stick.getRawButton(8)) {
                    // m_test.setSelectedSensorPosition(0);

                    // }
      // lime light 
      jas;




  }// Fin du teleop.periodic

  @Override
  public void autonomousInit() {
    // fakeahrs = new FakeAHRS(SPI.Port.kMXP); 
    // fakeahrs.YawMotors(m_leftMotor, m_rightMotor);
    // double pidOut;
  } // fin de autonomousInit

  @Override
  public void autonomousPeriodic() {

    m_distance = m_leftMotor.getSelectedSensorPosition();
    anglemesure = ahrs.getYaw();
    double pidOut;

    // // partie autonome
    // double m_distance = m_test.getSelectedSensorPosition();
    // SmartDashboard.putNumber("m_distance", m_distance);
    // double anglemesure = ahrs.getYaw();
    // int index = 1;

    // // SequencesAutonome TOURNERGAUCHE;

    // switch (etape) {

    // case 0:

    // if (jamaisattetint) {
    // if (m_distance < distanceautonome1) {
    // m_test.set(0.07);
    // } else {
    // m_test.set(0.0);
    // jamaisattetint = false;

    // etape = index++;
    // }
    // }

    // break;
    // case 1:
    // // Tourner a gauche 90 degrés
    // ahrs = new AHRS(SPI.Port.kMXP);
    // ahrs.reset();
    // m_pidController = new PIDController(kP, kI, kD, 0.02);
    // m_pidController.setSetpoint(90.0);

    // double pidOut = m_pidController.calculate(anglemesure);
    // m_robotDrive.arcadeDrive(0.0, pidOut);
    // etape = index++;
    // break;

    // case 2:
    // m_test.setSelectedSensorPosition(0);
    // if (jamaisattetint2) {
    // if (m_distance < distanceautonome2) {
    // m_test.set(0.07);

    // } else {
    // m_test.set(0.0);
    // jamaisattetint2 = false;
    // etape = index++;
    // }
    // }
    // break;

    // case 3:
    // ahrs = new AHRS(SPI.Port.kMXP);
    // ahrs.reset();
    // m_pidController = new PIDController(kP, kI, kD, 0.02);
    // m_pidController.setSetpoint(90.0);
    // m_pidController.calculate(anglemesure);
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // etape = index++;
    // break;

    // case 4:
    // m_test.setSelectedSensorPosition(0);
    // if (jamaisattetint4) {
    // if (m_distance < distanceautonome3) {
    // m_test.set(0.07);
    // } else {
    // m_test.set(0.0);
    // jamaisattetint4 = false;
    // }
    // }
    // break;

    // }

    // première routine

    // switch (step) {
    // case AVANCER_1:
    // if (initialiser) {
    // // par exemple reset position ou reset gyro
    // m_leftMotor.setSelectedSensorPosition(0);
    // m_distance = m_leftMotor.getSelectedSensorPosition();
    // initialiser = false;
    // }
    // if (m_distance > distanceautonome1) {
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // step = SequencesAutonome.TOURNER_1;
    // initialiser = true;
    // } else {
    // // avancer
    // m_robotDrive.arcadeDrive(0.5, 0.0);
    // }

    // break;
    // case TOURNER_1:
    // if (initialiser) {
    // // fakeahrs = new AHRS(SPI.Port.kMXP);
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // ahrs.reset();
    // m_pidController = new PIDController(kP, kI, kD, 0.02);
    // m_pidController.setSetpoint(90.0);
    // pidOut = m_pidController.calculate(anglemesure);
    // initialiser = false;
    // }
    // if (m_pidController.atSetpoint() | anglemesure > 89.5) { // this condition
    // may need to be removed
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // step = SequencesAutonome.AVANCER_2;
    // initialiser = true;
    // } else {
    // pidOut = m_pidController.calculate(anglemesure);
    // m_robotDrive.arcadeDrive(0.0, pidOut);
    // }

    // break;

    // case AVANCER_2:
    // if (initialiser) {
    // m_leftMotor.setSelectedSensorPosition(0);
    // m_distance = m_leftMotor.getSelectedSensorPosition();
    // initialiser = false;
    // }
    // if (m_distance > distanceautonome2) {
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // step = SequencesAutonome.TOURNER_2;
    // initialiser = true;
    // } else {
    // // avancer
    // m_robotDrive.arcadeDrive(0.5, 0.0);
    // }

    // break;

    // case TOURNER_2:
    // if (initialiser) {
    // // fakeahrs = new AHRS(SPI.Port.kMXP);
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // ahrs.reset();
    // anglemesure = ahrs.getYaw();
    // m_pidController = new PIDController(kP, kI, kD, 0.02);
    // m_pidController.setSetpoint(90.0);
    // pidOut = m_pidController.calculate(anglemesure);
    // initialiser = false;
    // }
    // if (m_pidController.atSetpoint() | anglemesure > 89.5) {
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // step = SequencesAutonome.AVANCER_3;
    // initialiser = true;
    // } else {
    // pidOut = m_pidController.calculate(anglemesure);
    // m_robotDrive.arcadeDrive(0.0, pidOut);
    // }

    // case AVANCER_3:
    // if (initialiser) {
    // m_leftMotor.setSelectedSensorPosition(0);
    // m_distance = m_leftMotor.getSelectedSensorPosition();
    // initialiser = false;
    // }
    // if (m_distance > distanceautonome3) {
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // step = SequencesAutonome.FIN;
    // initialiser = true;

    // } else {
    // // avancer
    // m_robotDrive.arcadeDrive(0.5, 0.0);
    // }

    // break;

    // case FIN:
    // m_pidController.close();
    // m_robotDrive.arcadeDrive(0.0, 0.0);

    // break;
    // default:

    // break;

    // }
    /// m_robotDrive.arcadeDrive(0.0, 0.0);

    // deuxième routine
    // initialiser = true;
    // switch (step2) {
    // case AVANCER_1:
    // if (initialiser) {
    // m_leftMotor.setSelectedSensorPosition(0);
    // m_distance = m_leftMotor.getSelectedSensorPosition();
    // initialiser = false;
    // }
    // if (m_distance > 99800) {
    // m_robotDrive.arcadeDrive(0.0, 0.0);
    // step2 = SequencesAutonome.LANCER;
    // initialiser = true;
    // } else {
    // m_robotDrive.arcadeDrive(0.5, 0.0);
    // }
    // break;
    // case LANCER:
    // m_shooter1.set(0.7);
    // m_shooter2.follow(m_shooter1);
    // break;
    // }

    // troisième routine
    initialiser = true;
    switch (step3) {
    case AVANCER_1:
      if (initialiser) {
        m_leftMotor.setSelectedSensorPosition(0);
        m_distance = m_leftMotor.getSelectedSensorPosition();
        initialiser = false;
      }
      if (m_distance > 41584) {
        m_robotDrive.arcadeDrive(0.0, 0.0);

      } else {
        m_robotDrive.arcadeDrive(0.5, 0.0);
      }

      break;

    default:
      break;
    }

  } // fin de l'autonome

} // fin du programme
