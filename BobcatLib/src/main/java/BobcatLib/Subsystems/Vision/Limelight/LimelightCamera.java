package BobcatLib.Subsystems.Vision.Limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import BobcatLib.Subsystems.Vision.Limelight.Components.PoseEstimator.LimelightPoseEstimator;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightData;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightResults;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightSettings;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightSettings.CamMode;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightSettings.LEDMode;
import BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightUtils;

import static BobcatLib.Subsystems.Vision.Limelight.Utility.LimelightUtils.getLimelightURLString;

/**
 * Limelight Camera class.
 */
public class LimelightCamera {

  /**
   * {@link Limelight} name.
   */
  public final String limelightName;
  /**
   * {@link Limelight} data from NetworkTables.
   */
  private LimelightData limelightData;
  /**
   * {@link Limelight} settings that we apply.
   */
  private LimelightSettings settings;


  /**
   * Constructs and configures the {@link Limelight} NT Values.
   *
   * @param name Name of the limelight.
   */
  public LimelightCamera(String name) {
    limelightName = name;
    limelightData = new LimelightData(this);
    settings = new LimelightSettings(this);
  }

  /**
   * Create a {@link LimelightPoseEstimator} for the {@link Limelight}.
   *
   * @param megatag2 Use MegaTag2.
   * @return {@link LimelightPoseEstimator}
   */
  public LimelightPoseEstimator getPoseEstimator(boolean megatag2) {
    return new LimelightPoseEstimator(this, megatag2);
  }

  /**
   * Get the {@link LimelightSettings} with current selections.
   */
  public LimelightSettings getSettings() {
    return settings;
  }

  /**
   * Get the {@link LimelightData} object for the {@link Limelight}
   *
   * @return {@link LimelightData} object.
   */
  public LimelightData getData() {
    return limelightData;
  }

  /**
   * Asynchronously take a snapshot in limelight.
   *
   * @param snapshotname Snapshot name to save.
   */
  public void snapshot(String snapshotname) {
    CompletableFuture.supplyAsync(() -> {
      URL url = getLimelightURLString(limelightName, "capturesnapshot");
      try {
        HttpURLConnection connection = (HttpURLConnection) url.openConnection();
        connection.setRequestMethod("GET");
        if (snapshotname != null && snapshotname != "") {
          connection.setRequestProperty("snapname", snapshotname);
        }

        int responseCode = connection.getResponseCode();
        if (responseCode == 200) {
          return true;
        } else {
          System.err.println("Bad LL Request");
        }
      } catch (IOException e) {
        System.err.println(e.getMessage());
      }
      return false;
    });
  }

  /**
   * Gets the latest JSON {@link LimelightResults} output and returns a
   * LimelightResults object.
   *
   * @return LimelightResults object containing all current target data
   */
  public Optional<LimelightResults> getLatestResults() {
    return limelightData.getResults();
  }

  /**
   * Flush the NetworkTable data to server.
   */
  public void flush() {
    NetworkTableInstance.getDefault().flush();
  }

  /**
   * Get the {@link NetworkTable} for this limelight.
   *
   * @return {@link NetworkTable} for this limelight.
   */
  public NetworkTable getNTTable() {
    return LimelightUtils.getNTTable(limelightName);
  }

  public void setLEDS(LEDMode mode) {
    switch (mode) {
      case ForceBlink:
        settings.withLimelightLEDMode(LEDMode.ForceBlink);
        break;
      case ForceOff:
        settings.withLimelightLEDMode(LEDMode.ForceOff);
      case ForceOn:
        settings.withLimelightLEDMode(LEDMode.ForceOn);
      case PipelineControl:
        settings.withLimelightLEDMode(LEDMode.PipelineControl);
      default:
        settings.withLimelightLEDMode(LEDMode.ForceOff);
        break;
    }
  }

  public void setCamMode(CamMode mode) {
    switch (mode) {
      case DriverCam:
        settings.withCamMode(CamMode.DriverCam);
      case Vision:
        settings.withCamMode(CamMode.DriverCam);
      default:
        settings.withCamMode(CamMode.DriverCam);
    }
  }

  public void setPipeline(int index) {
    settings.withPipelineIndex(index);
  }
}