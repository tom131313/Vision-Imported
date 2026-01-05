package frc.robot;

import org.opencv.core.MatOfDouble;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    /**
     * Vision constants for camera calibration and configuration
     */
    public static final class VisionConstants {

        /**
         * Camera calibration data for a specific camera configuration
         */
        public static class CameraCalibration {
            public final int width;
            public final int height;
            public final int fps;
            public final double fx;  // horizontal focal length, in pixels
            public final double fy;  // vertical focal length, in pixels
            public final double cx;  // horizontal focal center, in pixels
            public final double cy;  // vertical focal center, in pixels
            public final MatOfDouble distortionCoeffs;

            public CameraCalibration(int width, int height, int fps,
                                   double fx, double fy, double cx, double cy,
                                   MatOfDouble distortionCoeffs) {
                this.width = width;
                this.height = height;
                this.fps = fps;
                this.fx = fx;
                this.fy = fy;
                this.cx = cx;
                this.cy = cy;
                this.distortionCoeffs = distortionCoeffs;
            }
        }

        /**
         * ArduCam 320x240 calibration
         * Note: Rough calibration - wasn't done with a nice flat board.
         * Much of the small amount of distortion from calibration was actually the board
         * not being smooth so don't bother using the distortion.
         * Original distortion: [0.03872533667096114, -0.2121025605447465, 0.00334472765894009,
         *                       -0.006080540135581289, 0.4001779842036727]
         */
        public static final CameraCalibration ARDUCAM_320x240 = new CameraCalibration(
            320, 240, 100,
            273.8682279422785,
            274.2578211409246,
            142.187975375679,
            124.6151823259089,
            new MatOfDouble()
        );

        /**
         * ArduCam 1280x800 calibration
         * Camera: Arducam_OV9281_USB_Camera_(A?); Cameron board 1280x800
         * Assumes s1 s2 s3 s4 tx ty are all zeros
         */
        public static final CameraCalibration ARDUCAM_1280x800 = new CameraCalibration(
            1280, 800, 100,
            907.6920444758049,
            907.1513951038395,
            604.1750223777503,
            416.4609913313957,
            new MatOfDouble(
                0.040354289830866516, -0.044066115475547216, 6.662818829158613E-4, 9.755603732755772E-4, // k1 k2 p1 p2
                -0.013630390510289322, // k3
                -0.0011985508423857224, 0.003370423168524356, 0.0010337869630847195 // k4 k5 k6
            )
        );

        /**
         * LifeCam 320x240 calibration from PhotonVision
         */
        public static final CameraCalibration LIFECAM_320x240 = new CameraCalibration(
            320, 240, 30,
            353.74653217742724,
            340.77624878700817,
            163.5540798921191,
            119.8945718300403,
            new MatOfDouble()
        );

        /**
         * LifeCam 640x480 calibration from WPILib example
         * Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
         */
        public static final CameraCalibration LIFECAM_640x480 = new CameraCalibration(
            640, 480, 30,
            699.3778103158814,
            677.7161226393544,
            345.6059345433618,
            207.12741326228522,
            new MatOfDouble()
        );
    }
}
