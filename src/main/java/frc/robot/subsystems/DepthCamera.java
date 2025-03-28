package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.orbbec.obsensor.*;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;

public class DepthCamera extends SubsystemBase {
  /**
   * Object Declarations
   */
  private OBContext obContext;
  private final Object pipeLock = new Object();
  private Pipeline pipeline;

  private CvSource raw_stream;
  private CvSource ir_stream;
  private CvSource depth_stream;
  private CvSource depth_col_stream;

  private Mat raw_mat;
  private Mat ir_mat;
  private Mat depth_mat;
  private Mat depth_col_mat;
  private int gcol_r[] = new int[256];
  private int gcol_g[] = new int[256];
  private int gcol_b[] = new int[256];

  public DepthCamera() {
    initCamera();

    raw_stream = CameraServer.getInstance().putVideo("RAW", 640, 360);
    ir_stream = CameraServer.getInstance().putVideo("IR", 640, 480);
    depth_stream = CameraServer.getInstance().putVideo("DEPTH", 640, 360);
    depth_col_stream = CameraServer.getInstance().putVideo("DEPTH COLOR", 640, 360);

    raw_mat = new Mat(360, 640, CvType.CV_8UC3);
    ir_mat = new Mat(480, 640, CvType.CV_8U);
    depth_mat = new Mat(360, 640, CvType.CV_8U);
    depth_col_mat = new Mat(360, 640, CvType.CV_8UC3);

    double dk = 255.0 / 63.0;
    double ddat = 0.0;
    int c = 0;
    ddat = 0.0;
    for (int i = 0; i < 64; i++) {
      gcol_r[c] = 0;
      gcol_g[c] = (int) (ddat);
      gcol_b[c] = 255;
      ddat = ddat + dk;
      c++;
    }
    gcol_r[0] = 0;
    gcol_g[0] = 0;
    gcol_b[0] = 0;

    ddat = 255.0;
    for (int i = 64; i < 128; i++) {
      gcol_r[c] = 0;
      gcol_g[c] = 255;
      gcol_b[c] = (int) (ddat);
      ddat = ddat - dk;
      c++;
    }
    ddat = 0.0;
    for (int i = 128; i < 192; i++) {
      gcol_r[c] = (int) (ddat);
      gcol_g[c] = 255;
      gcol_b[c] = 0;
      ddat = ddat + dk;
      c++;
    }
    ddat = 255.0;
    for (int i = 192; i < 256; i++) {
      gcol_r[c] = 255;
      gcol_g[c] = (int) (ddat);
      gcol_b[c] = 0;
      ddat = ddat - dk;
      c++;
    }
  }

  private void initCamera() {
    // Init ObContext and regist the device status listnener
    obContext = new OBContext(new DeviceChangedCallback() {
      @Override
      public void onDeviceAttach(DeviceList deviceList) {
        // New devices attached
        // init the pipeline
        initPipeline(deviceList);
        startStreams();

        // Release the device list.
        deviceList.close();
      }

      @Override
      public void onDeviceDetach(DeviceList deviceList) {
        // Devices detached
        // Release the pipeline when the device is detached
        stopStreams();
        deInitPipeline();

        // Release the device list.
        deviceList.close();
      }
    });

    DeviceList deviceList = obContext.queryDevices();
    if (null != deviceList) {
      if (deviceList.getDeviceCount() > 0) {
        initPipeline(deviceList);
        startStreams();
      }
      deviceList.close();
    }
  }

  /**
   * Release the cameara when not used anymore
   */
  private void destroyCamera() {
    try {
      if (null != obContext) {
        obContext.close();
        obContext = null;
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  /**
   * Init the pipeline
   *
   * @param deviceList new attached device list
   */
  private void initPipeline(DeviceList deviceList) {
    synchronized (pipeLock) {
      if (null != pipeline) {
        pipeline.close();
      }

      // Create the new attached device, default for index 0
      try (Device device = deviceList.getDevice(0)) {
        pipeline = new Pipeline(device);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Release the pipeline
   */
  private void deInitPipeline() {
    synchronized (pipeLock) {
      try {
        if (null != pipeline) {
          pipeline.close();
          pipeline = null;
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Start streams, this function shows for streams config and starting streams by
   * pipeline
   */
  public void startStreams() {
    synchronized (pipeLock) {
      if (null == pipeline) {
        System.out.println("Camera Failed! No device connected!");
        return;
      }

      // Create the config for starting streams
      Config config = new Config();

      // Config for color stream
      try (StreamProfileList colorProfileList = pipeline.getStreamProfileList(SensorType.COLOR)) {
        // Filter out the StreamProfile you want based on specified arguments.
        VideoStreamProfile colorProfile =
            colorProfileList.getVideoStreamProfile(640, 360, Format.RGB888, 30);
        if (null == colorProfile) {
          throw new OBException(
              "get color stream profile failed, no matched color stream profile!");
        }
        // enable color stream
        config.enableStream(colorProfile);
        // release the color stream profile.
        colorProfile.close();
      } catch (Exception e) {
        e.printStackTrace();
      }

      // Config for depth stream.
      try (StreamProfileList depthProfileList = pipeline.getStreamProfileList(SensorType.DEPTH)) {
        // Filter out the StreamProfile you want based on specified arguments.
        // VideoStreamProfile depthProfile = depthProfileList.getVideoStreamProfile(640,
        // 480, Format.Y16, 30);
        VideoStreamProfile depthProfile = depthProfileList.getStreamProfile(0).as(StreamType.VIDEO);
        if (null == depthProfile) {
          throw new OBException(
              "get depth stream profile failed, no matched depth stream profile!");
        }
        // enable depth stream.
        config.enableStream(depthProfile);
        // release the depth stream profile.
        depthProfile.close();
      } catch (Exception e) {
        e.printStackTrace();
      }

      // Config for ir stream.
      try (StreamProfileList irProfileList = pipeline.getStreamProfileList(SensorType.IR)) {

        // Filter out the StreamProfile you want based on specified arguments.
        // VideoStreamProfile irProfile = irProfileList.getVideoStreamProfile(640, 480,
        // Format.Y16, 30);
        VideoStreamProfile irProfile = irProfileList.getStreamProfile(0).as(StreamType.VIDEO);
        if (null == irProfile) {
          throw new OBException("get ir stream profile failed, no matched ir streamm profile!");
        }
        // enable ir stream.
        config.enableStream(irProfile);
        // release the ir stream profile.
        irProfile.close();
      } catch (Exception e) {
        e.printStackTrace();
      }

      try {
        pipeline.start(config, new FrameSetCallback() {
          @Override
          public void onFrameSet(FrameSet frameSet) {
            // process the frame set
            processFrameSet(frameSet);
            frameSet.close();
          }
        });
        config.close();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  /**
   * Stop all the steams.
   */
  public void stopStreams() {
    synchronized (pipeLock) {
      try {
        if (null != pipeline) {
          pipeline.stop();
        }
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  private int[] depth_data = new int[640 * 360];

  private void disp_point_Depth_data(int x, int y, DepthFrame frame) {
    // Get Height and Width of Frame
    int width = frame.getWidth();
    int height = frame.getHeight(); // These are swapped in api

    // FOV of camera in Depth mode
    double fov_w = 79.0;
    double fov_h = 62.0;
    double pZ = (double) (depth_data[(y * width) + x]);

    // Calculate Theta W and H
    double theta_w = (fov_w / (double) width) * (x - (width / 2.0));
    double theta_h = (fov_h / (double) height) * (y - (height / 2.0));

    // Calculate X and Y distance in 3d notation
    double pX = pZ * Math.tan(Math.toRadians(theta_w));
    double pY = pZ * Math.tan(Math.toRadians(theta_h));

    // Build demo string to display data on a dashboard
    if (pZ > 0) {
      StringBuilder sb = new StringBuilder()
          .append("(" + x + "." + y + ") " + "x: " + Math.round(pX) + "mm, y: " + Math.round(pY)
              + "mm, z: "
              + pZ + "mm");
      SmartDashboard.putString("Data: ", sb.toString());
    }
  }

  private void Conv_Depth_Data(DepthFrame frame) {
    int width = frame.getWidth();
    int height = frame.getHeight();
    int x, y, i, ii;
    byte[] frameData = new byte[frame.getDataSize()];
    depth_data = new int[width * height];

    frame.getData(frameData);
    i = 0;
    ii = 0;
    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
        int dat1 = frameData[i];
        int dat2 = frameData[i + 1];
        int dat = ((dat2 << 8) & 0xFF00) + (dat1 & 0xFF);
        depth_data[ii] = dat;
        i = i + 2;
        ii = ii + 1;
      }
    }

  }

  private void Conv_Depth_Frame(DepthFrame frame) {
    int width = frame.getWidth();
    int height = frame.getHeight();
    int x, y, i, ii;
    byte[] depth_frameData = new byte[width * height];
    byte[] depth_color_frameData = new byte[width * height * 3];

    i = 0;
    ii = 0;
    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
        int dat = depth_data[i] - 100;
        if (dat < 0)
          dat = 0;
        dat = (dat >> 3);
        if (dat > 0xff)
          dat = 0xff;
        depth_color_frameData[ii] = (byte) (gcol_b[dat] & 0xff);
        depth_color_frameData[ii + 1] = (byte) (gcol_g[dat] & 0xff);
        depth_color_frameData[ii + 2] = (byte) (gcol_r[dat] & 0xff);
        i = i + 1;
        ii = ii + 3;
      }
    }
    depth_col_mat.put(0, 0, depth_color_frameData);

    i = 0;
    for (y = 0; y < height; y++) {
      for (x = 0; x < width; x++) {
        int dat = depth_data[i];
        dat = dat / 10;
        if (dat > 0xff)
          dat = 0xff;
        depth_frameData[i] = (byte) (dat & 0xff);
        i = i + 1;
      }
    }
    i = 0;
    ii = 0;
    for (y = height - 20; y < height - 10; y++) {
      i = (width - 256 * 2) / 2;
      for (x = 0; x < 255; x++) {
        ii = y * width + i;
        depth_frameData[ii] = (byte) x;
        depth_frameData[ii + 1] = (byte) x;
        i = i + 2;
      }
    }

    depth_mat.put(0, 0, depth_frameData);
  }

  private void Conv_Ir_Frame(IRFrame frame) {
    int height = frame.getHeight();
    int width = frame.getWidth();
    byte[] frameData = new byte[frame.getDataSize()];
    byte[] ir_frameData = new byte[width * height];
    frame.getData(frameData);
    int i = 0;
    int ii = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int dat1 = frameData[i];
        int dat2 = frameData[i + 1];
        int dat = ((dat2 << 8) & 0xFF00) + (dat1 & 0xFF);
        if (dat > 0xFF)
          dat = 0xff;
        ir_frameData[ii] = (byte) dat;
        i = i + 2;
        ii = ii + 1;
      }
    }
    ir_mat.put(0, 0, ir_frameData);
  }

  private void Conv_Color_Frame(ColorFrame frame) {
    // Get Height and Width of Frame
    int height = frame.getHeight();
    int width = frame.getWidth();
    byte[] frameData = new byte[frame.getDataSize()];
    byte[] raw_frameData = new byte[frame.getDataSize()];
    frame.getData(frameData);

    int i = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        raw_frameData[i] = frameData[i + 2];
        raw_frameData[i + 1] = frameData[i + 1];
        raw_frameData[i + 2] = frameData[i];
        i = i + 3;
      }
    }
    raw_mat.put(0, 0, raw_frameData);
  }

  /**
   * Process the frame set
   *
   * @param frameSet new frame set
   */
  private void processFrameSet(FrameSet frameSet) {

    // Get the color frame in frameSet.
    try (ColorFrame colorFrame = frameSet.getFrame(FrameType.COLOR)) {
      if (null != colorFrame) {
        Conv_Color_Frame(colorFrame);
        raw_stream.putFrame(raw_mat);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    // Get the depth frame in frameSet.
    try (DepthFrame depthFrame = frameSet.getFrame(FrameType.DEPTH)) {
      if (null != depthFrame) {
        Conv_Depth_Data(depthFrame);
        Conv_Depth_Frame(depthFrame);
        int px = 50, py = 150;

        disp_point_Depth_data(px, py, depthFrame);
        Scalar color = new Scalar(255, 255, 255);
        Imgproc.line(depth_mat, new Point(px - 10, py), new Point(px + 10, py), color);
        Imgproc.line(depth_mat, new Point(px, py - 10), new Point(px, py + 10), color);

        Imgproc.applyColorMap(depth_mat, depth_col_mat, Imgproc.COLORMAP_JET);
        depth_stream.putFrame(depth_mat);
        depth_col_stream.putFrame(depth_col_mat);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
    // Get the ir frame in frameSet.
    try (IRFrame irFrame = frameSet.getFrame(FrameType.IR)) {
      if (null != irFrame) {
        Conv_Ir_Frame(irFrame);
        ir_stream.putFrame(ir_mat);
      }
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
