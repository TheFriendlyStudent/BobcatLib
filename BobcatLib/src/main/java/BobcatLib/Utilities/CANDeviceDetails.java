package BobcatLib.Utilities;

public class CANDeviceDetails {
  public enum Manufacturer {
    Unknown,
    Thrifty,
    Grapple,
    Pwf,
    Redux,
    Rev,
    Ctre
  }

  private final Manufacturer manufacturer;
  private final int deviceNumber;
  private final String bus;

  /**
   * CAN Device Id constructor given the device number, bus name , and manufacturer type.
   *
   * @param deviceNumber
   * @param bus
   * @param manufacturer
   */
  public CANDeviceDetails(int deviceNumber, String bus, Manufacturer manufacturer) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
    this.manufacturer = manufacturer;
  }

  /**
   * CAN Device Id constructor given only the device number Uses the default bus name of "" (empty
   * string)
   *
   * @param deviceNumber
   */
  public CANDeviceDetails(int deviceNumber, Manufacturer manufacturer) {
    this(deviceNumber, "", manufacturer);
  }

  /**
   * CAN Device Id constructor given only the device number Uses the default bus name of "" (empty
   * string) and Unknown manufacturer
   *
   * @param deviceNumber
   */
  public CANDeviceDetails(int deviceNumber) {
    this(deviceNumber, "", Manufacturer.Unknown);
  }

  /**
   * Gets the manufacturer type
   *
   * @return Manufacturer type.
   */
  public Manufacturer getManufacturer() {
    return manufacturer;
  }

  /**
   * Gets the device number
   *
   * @return device number
   */
  public int getDeviceNumber() {
    return deviceNumber;
  }

  /**
   * Gets the bus name
   *
   * @return bus name
   */
  public String getBus() {
    return bus;
  }

  public boolean equals(CANDeviceDetails other) {
    return other.deviceNumber == deviceNumber
        && other.bus == bus
        && other.manufacturer == manufacturer;
  }
}
