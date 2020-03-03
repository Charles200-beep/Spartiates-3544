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
import edu.wpi.first.wpilibj.Servo;

enum Sequences {

  AVANCER_1, TOURNER_1, TOURNER_2, AVANCER_2, VISER, LANCER, FIN, AVANCER_3;

}

// Définir les composantes
public class Robot extends TimedRobot {
  private PIDController m_pidController;
  private WPI_TalonFX m_lanceur;
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
  private WPI_TalonFX m_test;
  private AHRS ahrs;
  private static final double kP = -.075;
  // private static final double kP2 = -.075;
  private static final double kI = -0.00;
  private static final double kD = -0.0;
  // private static final double ratioGearboxRoues = 8.68;
  // private static final double diametreRoues = 15.24; // en centimetres
  // private static final double rotationNombre = 2048;
  double distanceautonome1 = 1128726;
  double distanceautonome2 = 519808;
  double distanceautonome3 = 853970;

  // Initialisation automatique
  boolean initialiser = true;
  double distance = 0.0;
  Sequences step = Sequences.AVANCER_1;

  // private static final double kD2 = -0.0;
  boolean jamaisattetint = true;
  boolean jamaisattetint2 = true;
  boolean jamaisattetint3 = true;
  boolean jamaisattetint4 = true;
  boolean jamaisattetintroule = false;
  private DigitalInput feederhigh;
  private DigitalInput feederlow;
  int etape = 0;
  boolean feeder = true;
  boolean conveyor1 = true;
  boolean shoot = true;
  // int direction = m_stick.getPOV(0);
  // private Servo m_leftClimbRatchet;
  // private Servo m_rightClimbRatchet;
  double angle = 0.0;
  private Joystick m_stick2;

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
    m_intakeRoller = new WPI_VictorSPX(7);
    // m_rightClimbRatchet = new Servo(0);
    // m_leftClimbRatchet = new Servo(1);
    m_stick2 = new Joystick(1);
    m_lanceur = new WPI_TalonFX(-1);// changer
    m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_stick = new Joystick(0);
    ahrs = new AHRS(SPI.Port.kMXP);
    m_pidController = new PIDController(kP, kI, kD, 0.02);
    // m_pidController2 = new PIDController(kP2, kI, kD2, 0.02);
    m_pidController.setSetpoint(90.0);
    // m_pidController2.setSetpoint(0.0);
    m_test.setSelectedSensorPosition(0);
    c = new Compressor(0);
    // Faire suivre les autres moteurs
    m_leftMotor2.follow(m_leftMotor);
    m_rightMotor2.follow(m_rightMotor);

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
    double leftTrigger = m_stick.getRawAxis(4);
    double rightTrigger = m_stick.getRawAxis(5);
    double rotation = leftTrigger - rightTrigger;
    m_robotDrive.arcadeDrive(m_stick.getY(), rotation);

    // if (leftTrigger > -0.9 ) {
    // m_robotDrive.arcadeDrive(0, -leftTrigger);
    // }

    // if (rightTrigger > -0.9 ) {
    // m_robotDrive.arcadeDrive(0, -rightTrigger);
    // }

    // -------------------------------------------
    // détecter la chaleur des moteurs
    double m_temperature = m_test.getTemperature();
    refroidirMoteurs(m_temperature);
    SmartDashboard.putNumber("m_temperature", m_temperature);

    // Lire les données du limelight
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    // -------------------------------------------
    // limit switches intake arm
  
    double a = m_stick.getRawAxis(3);
    double b;

    if (a < 0.0 & intakeArmLow.get() == true) {
      b = 0.0;
    } else {
      if (a > 0.0 & intakeArmHigh.get() == true) {
        b = 0.0;
      } else {
        b = a;
      }
    }

    m_intakeArm.set(b);

    // --------------------------------------------

    // Intake Roller
    if (m_stick2.getRawButton(4)) {// changer
      m_intakeRoller.set(0.7);
    } else {
      m_intakeRoller.set(0.0);
    }

    // -------------------------------------------
    // limit switches climb
    double z = m_stick.getRawAxis(3);
    double d;
    if (z < 0.0 & leftClimbStop.get() == true) {
      d = 0.0;
    } else {
      if (z > 0.0 & leftClimbStop.get() == true) {
        d = 0.0;
      } else {
        d = z;
      }
    }

    m_leftClimb.set(d);

    // --------------------------------------------
    // limit switches climb
    double e = m_stick.getRawAxis(3);
    double f;
    if (e < 0.0 & rightClimbStop.get() == true) {
      f = 0.0;
    } else {
      if (e > 0.0 & rightClimbStop.get() == true) {
        f = 0.0;
      } else {
        f = e;
      }
    }
    m_rightClimb.set(f);
    // ----------------------------------------------

    // shooter
    if (m_stick2.getRawButtonReleased(1) & shoot == true) {
      m_shooter1.set(0.4);
      m_shooter2.follow(m_shooter1);
      shoot = false;

    }
    if (m_stick2.getRawButtonReleased(1) & shoot == false) {
      m_shooter1.set(-0.4);
      m_shooter2.follow(m_shooter1);
      shoot = true;

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

    // ---------------------------------------------------
    // conveyor
    if (m_stick2.getRawButton(2) & conveyor1 == true) {
      m_conveyorHigh.set(0.7);
      m_conveyorLow.follow(m_conveyorHigh);
      conveyor1 = false;

      if (m_stick2.getRawButtonReleased(2)) {
        m_conveyorHigh.stopMotor();
        m_conveyorLow.follow(m_conveyorHigh);
        conveyor1 = true;
      }
    }

    // ----------------------------------------------------
    // feeder
    if (m_stick.getRawButtonReleased(6) & feeder == true & feederhigh.get() == true & feederlow.get() == false) {
      m_feederBall.set(0.9);
      feeder = false;

    }
    if (m_stick.getRawButtonReleased(6) & feeder == false & feederlow.get() == true & feederhigh.get() == false) {
      m_feederBall.set(-0.9);
      feeder = true;
    }
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
    SmartDashboard.putNumber("Limelightx", x);
    SmartDashboard.putNumber("LimelightArea", area);

    // SmartDashboard.putNumber("pidOut2", pidOut2);

    // Système de controle automatique
    if (m_stick2.getRawButton(3))
      suivreBalle(x);

    // Tourner a un angle
    if (m_stick.getRawButton(7) & m_stick.getRawButton(8))
      ahrs.reset();
    // faire
    // m_pidController2.setSetpoint(0.0);

    if (m_stick.getRawButton(5) | m_stick.getRawButton(6)) {
      // double erreur = 90.0 - anglemesure;
      m_robotDrive.arcadeDrive(0.0, pidOut);
    }

    // if (m_stick.getRawButton(4)) {
    // m_robotDrive.arcadeDrive(-0.7, pidOut2);

    if (m_stick.getRawButton(7) & m_stick.getRawButton(8)) {
      m_test.setSelectedSensorPosition(0);

    }

  }// Fin du teleop.periodic

  public void autonomousPeriodic() {

    double m_distance = m_test.getSelectedSensorPosition();
    SmartDashboard.putNumber("m_distance", m_distance);
    double anglemesure = ahrs.getYaw();
    // // partie autonome
    // double m_distance = m_test.getSelectedSensorPosition();
    // SmartDashboard.putNumber("m_distance", m_distance);
    // double anglemesure = ahrs.getYaw();
    // int index = 1;

    // // Sequences TOURNERGAUCHE;

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

    switch (step) {
    case AVANCER_1:
      if (initialiser) {
        // par exemple reset position ou reset gyro
        m_test.setSelectedSensorPosition(0);
        initialiser = false;
      }
      if (m_distance > distanceautonome1) {
        m_test.set(0.0);
        step = Sequences.TOURNER_1;
        initialiser = true;
      } else {
        // avancer
        m_test.set(0.07);
      }

      break;
    case TOURNER_1:
      if (initialiser) {
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();
        m_pidController = new PIDController(kP, kI, kD, 0.02);
        m_pidController.setSetpoint(90.0);
        initialiser = false;
      }
      if (m_pidController.atSetpoint()) {
        step = Sequences.AVANCER_2;
        initialiser = true;
      } else {
        double pidOut = m_pidController.calculate(anglemesure);
        m_robotDrive.arcadeDrive(0.0, pidOut);
      }

      break;

    case AVANCER_2:
      if (initialiser) {
        m_test.setSelectedSensorPosition(0);
        initialiser = false;
      }
      if (m_distance > distanceautonome2) {
        m_test.set(0.0);
        step = Sequences.TOURNER_2;
        initialiser = true;
      } else {
        // avancer
        m_test.set(0.07);
      }

      break;

    case TOURNER_2:
      if (initialiser) {
        ahrs = new AHRS(SPI.Port.kMXP);
        ahrs.reset();
        m_pidController = new PIDController(kP, kI, kD, 0.02);
        m_pidController.setSetpoint(90.0);
        initialiser = false;
      }
      if (m_pidController.atSetpoint()) {
        step = Sequences.AVANCER_3;
        initialiser = true;
      } else {
        double pidOut = m_pidController.calculate(anglemesure);
        m_robotDrive.arcadeDrive(0.0, pidOut);
      }

      break;

    case AVANCER_3:
      if (initialiser) {
        m_test.setSelectedSensorPosition(0);
        initialiser = false;
      }
      if (m_distance > distanceautonome3) {
        m_test.set(0.0);
        step = Sequences.FIN;
        initialiser = true;

      } else {
        // avancer
        m_test.set(0.07);
      }

      break;

    case FIN:

    break;
    default:

      break;

    }

  } // fin de l'autonome

} // fin du programme
