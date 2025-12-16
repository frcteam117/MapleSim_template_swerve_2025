package frc.robot.util;

import edu.wpi.first.math.util.Units;

public class UnitUtil {
  /**
   * Converts given meters to feet.
   *
   * @param value_m The meters to convert to feet.
   * @return Feet converted from meters.
   */
  public static double mToft(double value_m) {
    return Units.metersToFeet(value_m);
  }

  /**
   * Converts given feet to meters.
   *
   * @param value_ft The feet to convert to meters.
   * @return Meters converted from feet.
   */
  public static double ftTom(double value_ft) {
    return Units.feetToMeters(value_ft);
  }

  /**
   * Converts given meters to inches.
   *
   * @param value_m The meters to convert to inches.
   * @return Inches converted from meters.
   */
  public static double mToin(double value_m) {
    return Units.metersToInches(value_m);
  }

  /**
   * Converts given inches to meters.
   *
   * @param value_in The inches to convert to meters.
   * @return Meters converted from inches.
   */
  public static double inTom(double value_in) {
    return Units.inchesToMeters(value_in);
  }

  /**
   * Converts given degrees to radians.
   *
   * @param value_deg The degrees to convert to radians.
   * @return Radians converted from degrees.
   */
  public static double degTorad(double value_deg) {
    return Units.degreesToRadians(value_deg);
  }

  /**
   * Converts given radians to degrees.
   *
   * @param value_rad The radians to convert to degrees.
   * @return Degrees converted from radians.
   */
  public static double radTodeg(double value_rad) {
    return Units.radiansToDegrees(value_rad);
  }

  /**
   * Converts given radians to rotations.
   *
   * @param value_rad The radians to convert.
   * @return rotations Converted from radians.
   */
  public static double radTorot(double value_rad) {
    return Units.radiansToRotations(value_rad);
  }

  /**
   * Converts given degrees to rotations.
   *
   * @param value_deg The degrees to convert.
   * @return rotations Converted from degrees.
   */
  public static double degTorot(double value_deg) {
    return Units.degreesToRotations(value_deg);
  }

  /**
   * Converts given rotations to degrees.
   *
   * @param value_rot The rotations to convert.
   * @return degrees Converted from rotations.
   */
  public static double rotTodeg(double value_rot) {
    return Units.rotationsToDegrees(value_rot);
  }

  /**
   * Converts given rotations to radians.
   *
   * @param value_rot The rotations to convert.
   * @return radians Converted from rotations.
   */
  public static double rotTorad(double value_rot) {
    return Units.rotationsToRadians(value_rot);
  }

  /**
   * Converts rotations per minute to radians per second.
   *
   * @param value_RPM The rotations per minute to convert to radians per second.
   * @return Radians per second converted from rotations per minute.
   */
  public static double RPMToradPs(double value_RPM) {
    return Units.rotationsPerMinuteToRadiansPerSecond(value_RPM);
  }

  /**
   * Converts radians per second to rotations per minute.
   *
   * @param value_radPs The radians per second to convert to from rotations per minute.
   * @return Rotations per minute converted from radians per second.
   */
  public static double radPsToRPM(double value_radPs) {
    return Units.radiansPerSecondToRotationsPerMinute(value_radPs);
  }

  /**
   * Converts given milliseconds to seconds.
   *
   * @param value_ms The milliseconds to convert to seconds.
   * @return Seconds converted from milliseconds.
   */
  public static double msTos(double value_ms) {
    return Units.millisecondsToSeconds(value_ms);
  }

  /**
   * Converts given seconds to milliseconds.
   *
   * @param value_s The seconds to convert to milliseconds.
   * @return Milliseconds converted from seconds.
   */
  public static double sToms(double value_s) {
    return Units.secondsToMilliseconds(value_s);
  }

  /**
   * Converts kilograms into lbs (pound-mass).
   *
   * @param value_kg The kilograms to convert to lbs (pound-mass).
   * @return Lbs (pound-mass) converted from kilograms.
   */
  public static double kgTolb(double value_kg) {
    return Units.kilogramsToLbs(value_kg);
  }

  /**
   * Converts lbs (pound-mass) into kilograms.
   *
   * @param value_lb The lbs (pound-mass) to convert to kilograms.
   * @return Kilograms converted from lbs (pound-mass).
   */
  public static double lbTokg(double value_lb) {
    return Units.lbsToKilograms(value_lb);
  }
}
