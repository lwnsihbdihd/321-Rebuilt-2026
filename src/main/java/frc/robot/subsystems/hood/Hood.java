/* (C) RoboLancers 2026 */
package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.DigitalInputsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;

public class Hood extends SubsystemBase {

  // Creates objects of the TalonFX, DigitalInput, and CANdi classes.
  @Logged private TalonFX hoodMotor = new TalonFX(HoodConstants.kHoodMotorId);
  @Logged private DigitalInput hoodLimitSwitch = new DigitalInput(HoodConstants.kLimitSwitchID);
  @Logged private CANdi magneticLimitSwitch = new CANdi(0);

  // Makes a targetAngle variable set to kStartingAngle to make it more easily accessible.
  private Angle targetAngle = HoodConstants.kStartingAngle;

  // Makes a new PIDController object.
  public PIDController pidController = new PIDController(0, 0, 0);

  public ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0);

  public Hood() {
    configureMotors();

    // Sets the values of the hood PID.
    setHoodPID(HoodConstants.kP, HoodConstants.kD, HoodConstants.kG);
    magneticLimitSwitch.getConfigurator().apply(candiConfigs);
  }

  public void configureMotors() {

    TalonFXConfiguration hoodMotorConfigs =
        new TalonFXConfiguration()
          // Sets current limits to protect hardware.
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(HoodConstants.kHoodStatorCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(HoodConstants.kHoodSupplyCurrentLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        HoodConstants.kHoodMotorInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(HoodConstants.kHoodMotorMaxVelocity)
                    .withMotionMagicAcceleration(HoodConstants.kHoodMotorMaxAcceleration))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
            .withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(HoodConstants.kHoodGearRatio));

    // Applies the motor configs defined above.
    hoodMotor.getConfigurator().apply(hoodMotorConfigs);
  }

  // Sets specific CANdi configs.
  public CANdiConfiguration candiConfigs =
      new CANdiConfiguration()
          .withDigitalInputs(
              new DigitalInputsConfigs().withS1CloseState(S1CloseStateValue.CloseWhenLow));

  // Method to easily change the hood PID values.
  public void setHoodPID(double kP, double kD, double kG) {
    pidController.setP(kP);
    pidController.setD(kD);
    armFeedforward.setKg(kG);
  }

  // Method to easily change the target angle by setting the targetAngle variable to an angle set in the parameters.
  public void setTargetAngle(Angle angle) {
    this.targetAngle = angle;
  }

  // Method to move the motor to a certain angle.
  public void goToAngle(Angle angle) {
    hoodMotor.setVoltage(
        pidController.calculate(getAngle().in(Degrees), angle.in(Degrees))
            + armFeedforward.calculate(angle.in(Degrees), 0));
  }

  // Method to calculate the angle the motor needs to get to to score in the hub.
  public Angle getScoreAngle(Distance hubDistance) {
    double pitch = 0;
    pitch = -37.69619 + 9.35374 * Math.log(hubDistance.in(Inches));
    return Degrees.of(pitch);
  }

  // Method that returns the current angle of the motor.
  @Logged(name = "hoodPitch")
  public Angle getAngle() {
    Angle angle = hoodMotor.getPosition().getValue();
    return angle;
  }

  // Method that returns the angular velocity of the motor.
  @Logged(name = "hoodVelocity")
  public AngularVelocity getVelocity() {
    return hoodMotor.getVelocity().getValue();
  }

  // Method that checks if homed velocity is within a set tolerance value.
  public boolean isHomedVelocity() {
    return Math.abs(getVelocity().in(RPM) - HoodConstants.kHomingVelocityFloor.in(RPM))
        <= HoodConstants.kHomingVelocityTolerance;
  }

  // Method that checks if homed current is above or equal to a set ceiling value for the current.
  public boolean isHomedCurrent() {
    return getCurrent().in(Amps) >= HoodConstants.kCurrentCeiling.in(Amps);
  }

  // Method that checks if S1 input matches the configured S1 closed state.
  public boolean atHomedPosition() {
    return magneticLimitSwitch.getS1Closed(true).getValue();
  }

  // Method that zeros the encoder by setting the position of the motor to specific value.
  public void zeroEncoder() {
    hoodMotor.setPosition(HoodConstants.kZeroPosition);
  }

  // Method to run volts through the motor.
  public void runVolts(Voltage volts) {
    hoodMotor.setVoltage(volts.in(Volts));
  }

  // Method to check if motor is at the target angle.
  @Logged(name = "hoodAtTargetAngle")
  public boolean atTargetAngle() {
    boolean atTargetAngle =
        Math.abs(getAngle().in(Degrees) - targetAngle.in(Degrees))
            < HoodConstants.kAngleTolerance.in(Degrees);
    return atTargetAngle;
  }

  // Method that checks the current angle of the motor.
  public boolean atAngle(Angle angle) {
    boolean atAngle =
        Math.abs(getAngle().in(Degrees) - angle.in(Degrees))
            < HoodConstants.kAngleTolerance.in(Degrees);
    return atAngle;
  }

  // Method to set the hood PID and move the motor to the target angle.
  public void tune(double kP, double kD, double kG, double targetAngle) {
    setHoodPID(kP, kD, kG);
    goToAngle(Degrees.of(targetAngle));
  }

  TunableConstant kHomingVoltage = new TunableConstant("Homing voltage", 0);

  public Command homeHoodMagnetic() {
    return run(() -> hoodMotor.setVoltage(kHomingVoltage.get()))
        .until(() -> atHomedPosition())
        .andThen(() -> zeroEncoder());
  }

  // Method that returns the target angle.
  @Logged(name = "hoodTargetAngle")
  public Angle getTargetAngle() {
    return this.targetAngle;
  }

  // Method to get the current voltage of the motor.
  @Logged(name = "hoodVoltage")
  public Voltage getVoltage() {
    return hoodMotor.getMotorVoltage().getValue();
  }

  // Method to get the current current of the motor.
  @Logged(name = "hoodCurrent")
  public Current getCurrent() {
    return hoodMotor.getStatorCurrent().getValue();
  }

  @Override
  public void periodic() {
    // Puts the hood angle, voltage, and if it is at homed position on SmartDashboard, allowing the values to be viewed live as it changes.
    SmartDashboard.putNumber("Hood Angle", hoodMotor.getPosition().getValue().in(Degrees));
    SmartDashboard.putBoolean("Hood Is At Homed Position", atHomedPosition());
    SmartDashboard.putNumber("Hood Voltage", getVoltage().in(Volts));
  }
}
