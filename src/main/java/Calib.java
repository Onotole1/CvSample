import components.Constants;
import components.ImageUtils;
import components.MatUtils;
import de.humatic.dsj.DSCapture;
import de.humatic.dsj.DSFilterInfo;
import de.humatic.dsj.DSFiltergraph;
import de.humatic.dsj.DSJException;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowEvent;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class Calib extends JFrame {

    private static final Size PATTERN_SIZE = new Size(10, 7);
    private static final Size FRAME_SIZE = new Size(640, 480);

    private static final double SQUARE_SIZE = 30;

    private final int mCornersSize = (int) (PATTERN_SIZE.width * PATTERN_SIZE.height);

    private JLabel label1;
    private JButton finishButton;
    private JPanel panel1;
    private JPanel panel2;
    private JPanel panel3;
    private JButton addButton;
    private JButton calibrateButton;
    private JButton saveButton;
    private JButton resetButton;
    private JTextArea calibInfo;
    private JSpinner alphaSpinner;
    private JComboBox<String> comboBoxRotation2;
    private JComboBox<String> comboBoxRotation1;
    private JLabel device2NotSupported;
    private JLabel device1NotSupported;
    private JList<String> list1;
    private JList<String> list2;
    private final ArrayList<DSFilterInfo> devices = new ArrayList<>();

    private Mat img1;
    private Mat img2;


    private final Mat gray1 = new Mat();
    private final Mat gray2 = new Mat();

    private final List<Mat> objectPoints = new ArrayList<>();
    private final List<Mat> imagePoints1 = new ArrayList<>();
    private final List<Mat> imagePoints2 = new ArrayList<>();

    private final Mat D1 = new Mat();
    private final Mat D2 = new Mat();
    private final Mat R = new Mat();
    private final Mat T = new Mat();
    private final Mat E = new Mat();
    private final Mat F = new Mat();
    private final Mat R1 = new Mat();
    private final Mat R2 = new Mat();
    private final Mat P1 = new Mat();
    private final Mat P2 = new Mat();
    private final Mat Q = new Mat();

    private final Mat CM1 = Mat.eye(3, 3, CvType.CV_64F);
    private final Mat CM2 = Mat.eye(3, 3, CvType.CV_64F);


    private static DSCapture cam1graph;
    private static DSCapture cam2graph;

    private final AtomicInteger selectedRotation1 = new AtomicInteger();
    private final AtomicInteger selectedRotation2 = new AtomicInteger();

    private final AtomicBoolean isCalibrationRun = new AtomicBoolean();
    private final AtomicBoolean isDemoRun = new AtomicBoolean();
    private final AtomicBoolean isAddPressed = new AtomicBoolean();

    private final Runnable depthRunnable = () -> {

        while (isCalibrationRun.get()) {

            try {
                readImages();
            } catch (RuntimeException e) {
                try {
                    Thread.sleep(2000);
                } catch (InterruptedException e1) {
                    //do nothing
                }

                continue;
            }

            Imgproc.cvtColor(img1, gray1, Imgproc.COLOR_BGR2GRAY);

            Imgproc.cvtColor(img2, gray2, Imgproc.COLOR_BGR2GRAY);

            final MatOfPoint2f corners1 = new MatOfPoint2f();
            final MatOfPoint2f corners2 = new MatOfPoint2f();

            final boolean found1 = Calib3d.findChessboardCorners(gray1, PATTERN_SIZE, corners1, Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_NORMALIZE_IMAGE);

            final boolean found2 = Calib3d.findChessboardCorners(gray2, PATTERN_SIZE, corners2, Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_NORMALIZE_IMAGE);

            if (found1) {
                Imgproc.cornerSubPix(gray1, corners1, new Size(11, 11), new Size(-1, -1), new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.001));
                Calib3d.drawChessboardCorners(gray1, PATTERN_SIZE, corners1, true);
            }

            if (found2) {
                Imgproc.cornerSubPix(gray2, corners2, new Size(11, 11), new Size(-1, -1), new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.001));
                Calib3d.drawChessboardCorners(gray2, PATTERN_SIZE, corners2, true);
            }

            ImageUtils.draw(panel1, gray1);
            ImageUtils.draw(panel2, gray2);

            if (found1 && found2) {
                addButton.setEnabled(true);

                if (isAddPressed.get() && !corners1.empty() && !corners2.empty()) {

                    final List<Mat> rvecs1 = new ArrayList<>();
                    final List<Mat> rvecs2 = new ArrayList<>();
                    final List<Mat> tvecs1 = new ArrayList<>();
                    final List<Mat> tvecs2 = new ArrayList<>();

                    imagePoints1.add(corners1);
                    imagePoints2.add(corners2);


                    final Mat zeros = Mat.zeros(mCornersSize, 1, CvType.CV_32FC3);

                    calcBoardCornerPositions(zeros);
                    objectPoints.add(zeros);

                    calibrateButton.setEnabled(true);

                    double rms1 = Calib3d.calibrateCamera(objectPoints, imagePoints1, gray1.size(), CM1, D1, rvecs1, tvecs1,
                            Calib3d.CALIB_FIX_PRINCIPAL_POINT
                                    | Calib3d.CALIB_FIX_ASPECT_RATIO
                                    | Calib3d.CALIB_ZERO_TANGENT_DIST
                                    | Calib3d.CALIB_RATIONAL_MODEL
                                    | Calib3d.CALIB_FIX_K3
                                    | Calib3d.CALIB_FIX_K4
                                    | Calib3d.CALIB_FIX_K5);

                    double rms2 = Calib3d.calibrateCamera(objectPoints, imagePoints2, gray2.size(), CM2, D2, rvecs2, tvecs2,
                            Calib3d.CALIB_FIX_PRINCIPAL_POINT
                                    | Calib3d.CALIB_FIX_ASPECT_RATIO
                                    | Calib3d.CALIB_ZERO_TANGENT_DIST
                                    | Calib3d.CALIB_RATIONAL_MODEL
                                    | Calib3d.CALIB_FIX_K3
                                    | Calib3d.CALIB_FIX_K4
                                    | Calib3d.CALIB_FIX_K5);

                    System.out.println("rms1 : " + rms1);
                    System.out.println("rms2 : " + rms2);

                    label1.setText("Calibrate images count: " + imagePoints1.size());

                    isAddPressed.set(false);
                }
            } else {
                addButton.setEnabled(false);
            }

        }
    };

    @Override
    protected void processWindowEvent(final WindowEvent e) {


        if (e.getID() == WindowEvent.WINDOW_CLOSING) {
            stopEasyCaps();
        }

        super.processWindowEvent(e);
    }

    Calib() {

        setContentPane(panel3);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        pack();

        final DSFilterInfo[][] info = DSCapture.queryDevices();

        for (final DSFilterInfo[] anInfo : info) {
            devices.addAll(Arrays.asList(anInfo));
        }

        final String[] devicesNames = new String[devices.size()];
        for (int i = 0, devicesSize = devices.size(); i < devicesSize; i++) {
            devicesNames[i] = devices.get(i).getName();
        }

        list1.setListData(devicesNames);
        list2.setListData(devicesNames);

        list1.addListSelectionListener(e -> {
            if (devices.size() > 0) {
                initEasyCap(devices.get(list1.getSelectedIndex()), 0);
            }
        });

        list2.addListSelectionListener(e -> {
            if (devices.size() > 0) {
                initEasyCap(devices.get(list2.getSelectedIndex()), 1);
            }
        });

        if (devices.size() > 0) {

            if (devices.size() > 1) {
                list1.setSelectedIndex(0);
                list2.setSelectedIndex(1);
            } else {
                list1.setSelectedIndex(0);
                list2.setSelectedIndex(0);

            }
        }

        comboBoxRotation1.addActionListener(e -> {
            for (final String key : Constants.ROTATIONS.keySet()) {
                if (key.equals(comboBoxRotation1.getSelectedItem())) {
                    selectedRotation1.set(Constants.ROTATIONS.get(key));
                }
            }
        });

        comboBoxRotation2.addActionListener(e -> {
            for (final String key : Constants.ROTATIONS.keySet()) {
                if (key.equals(comboBoxRotation2.getSelectedItem())) {
                    selectedRotation2.set(Constants.ROTATIONS.get(key));
                }
            }
        });

        for (final String key : Constants.ROTATIONS.keySet()) {
            comboBoxRotation1.addItem(key);
            comboBoxRotation2.addItem(key);
        }

        comboBoxRotation1.setSelectedIndex(0);
        comboBoxRotation2.setSelectedIndex(0);

        alphaSpinner.setModel(new SpinnerNumberModel(0d, Integer.MIN_VALUE, Integer.MAX_VALUE, 0.1d));

        resetButton.addActionListener(e -> {

            if (isDemoRun.get()) {
                isDemoRun.set(false);
                isCalibrationRun.set(true);

                new Thread(depthRunnable).start();
            }

            objectPoints.clear();
            imagePoints1.clear();
            imagePoints2.clear();

            label1.setText("Calibrate images count: " + imagePoints1.size());
            calibrateButton.setEnabled(false);
            saveButton.setEnabled(false);
        });

        addButton.addActionListener(e -> {
            if (addButton.isEnabled()) {
                isAddPressed.set(true);
            }
        });

        saveButton.addActionListener(e -> {
            if (imagePoints1.size() > 0 && saveButton.isEnabled()) {

                try {
                    saveCalibration();
                } catch (final IOException exception) {
                    //do nothing
                }
            }
        });

        calibrateButton.addActionListener(e -> {
            calibrate();

            saveButton.setEnabled(true);

            final Runnable demoRunnable = () -> {

                final Mat map1x = new Mat();
                final Mat map1y = new Mat();
                final Mat map2x = new Mat();
                final Mat map2y = new Mat();
                final Mat imgU1 = new Mat();
                final Mat imgU2 = new Mat();


                Imgproc.initUndistortRectifyMap(CM1, D1, R1, P1, FRAME_SIZE, CvType.CV_32FC1, map1x, map1y);
                Imgproc.initUndistortRectifyMap(CM2, D2, R2, P2, FRAME_SIZE, CvType.CV_32FC1, map2x, map2y);

                while (isDemoRun.get()) {

                    try {
                        readImages();
                    } catch (final RuntimeException exception) {
                        try {
                            Thread.sleep(2000);
                        } catch (final InterruptedException exception1) {
                            //do nothing
                        }

                        continue;
                    }

                    Imgproc.remap(img1, imgU1, map1x, map1y, Imgproc.INTER_LINEAR);
                    Imgproc.remap(img2, imgU2, map2x, map2y, Imgproc.INTER_LINEAR);

                    ImageUtils.draw(panel1, imgU1);
                    ImageUtils.draw(panel2, imgU2);

                }
            };

            isCalibrationRun.set(false);

            isDemoRun.set(true);

            new Thread(demoRunnable).start();
        });

        final Thread thread = new Thread(depthRunnable);

        isCalibrationRun.set(true);

        thread.start();

    }

    private void calcBoardCornerPositions(final Mat corners) {
        final int cn = 3;
        final float[] positions = new float[mCornersSize * cn];

        for (int i = 0; i < PATTERN_SIZE.height; i++) {
            for (int j = 0; j < PATTERN_SIZE.width * cn; j += cn) {
                positions[(int) (i * PATTERN_SIZE.width * cn + j + 0)] =
                        (2 * (j / cn) + i % 2) * (float) SQUARE_SIZE;
                positions[(int) (i * PATTERN_SIZE.width * cn + j + 1)] =
                        i * (float) SQUARE_SIZE;
                positions[(int) (i * PATTERN_SIZE.width * cn + j + 2)] = 0;
            }
        }
        corners.create(mCornersSize, 1, CvType.CV_32FC3);
        corners.put(0, 0, positions);
    }

    private void createUIComponents() {
        panel1 = new JPanel();
        panel2 = new JPanel();
        panel3 = new JPanel();
        label1 = new JLabel();
        finishButton = new JButton();
        finishButton.addActionListener(e -> {
            if (imagePoints1.size() > 0) {
                isCalibrationRun.set(false);

                final Main main = new Main();

                EventQueue.invokeLater(() ->
                        main.setVisible(true));

                dispatchEvent(new WindowEvent(this, WindowEvent.WINDOW_CLOSING));
            }
        });
    }

    private void initEasyCap(final DSFilterInfo easyCap, final int number) {

        try {

            switch (number) {

                case 0:

                    cam1graph = new DSCapture(DSFiltergraph.CAPTURE, easyCap,
                            false, DSFilterInfo.doNotRender(), null);

                    cam1graph.setVisible(true);
                    cam1graph.setPreview();
                    break;
                case 1:

                    cam2graph = new DSCapture(DSFiltergraph.CAPTURE, easyCap,
                            false, DSFilterInfo.doNotRender(), null);

                    cam2graph.setVisible(true);
                    cam2graph.setPreview();
                    break;
            }
        } catch (final DSJException e) {
            //do nothing
        }

    }

    private void stopEasyCaps() {
        cam1graph.stop();
        cam2graph.stop();
    }

    private void readImages() throws RuntimeException {

        final BufferedImage cam1graphImage;
        try {
            cam1graphImage = cam1graph.getImage();
            img1 = ImageUtils.bufferedImage2Mat(cam1graphImage);
        } catch (final NullPointerException e) {
            device1NotSupported.setVisible(true);
        }

        final BufferedImage cam2graphImage;
        try {
            cam2graphImage = cam2graph.getImage();
            img2 = ImageUtils.bufferedImage2Mat(cam2graphImage);
        } catch (final NullPointerException e) {
            device2NotSupported.setVisible(true);
        }

        if (img1 == null || img2 == null) {
            device1NotSupported.setVisible(img1 == null);
            device2NotSupported.setVisible(img2 == null);
            throw new RuntimeException();
        }

        final int rotation1 = selectedRotation1.get();
        if (rotation1 != Constants.NO_ROTATION_VALUE) {
            Core.flip(img1, img1, rotation1);
        }

        final int rotation2 = selectedRotation2.get();
        if (rotation2 != Constants.NO_ROTATION_VALUE) {
            Core.flip(img2, img2, rotation2);
        }

        if (!img1.size().equals(FRAME_SIZE)) {
            img1 = img1.submat(new Rect(new Point(0, 0), FRAME_SIZE));
        }

        if (!img2.size().equals(FRAME_SIZE)) {
            img2 = img2.submat(new Rect(new Point(0, 0), FRAME_SIZE));
        }

    }


    private void calibrate() {

        System.out.println(objectPoints.size() + " " + imagePoints1.size() + " " + imagePoints2.size());

        Calib3d.stereoCalibrate(objectPoints, imagePoints1, imagePoints2, CM1, D1, CM2, D2, img1.size(), R, T, E, F, Calib3d.CALIB_FIX_INTRINSIC | Calib3d.CALIB_ZERO_TANGENT_DIST, new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, 0));

        final Rect validPixROI1 = new Rect();
        final Rect validPixROI2 = new Rect();

        Calib3d.stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, Calib3d.CALIB_ZERO_DISPARITY, (Double) alphaSpinner.getValue(), gray1.size(), validPixROI1, validPixROI2);

        calibInfo.setText(MatUtils.getCalibInfo(CM1, CM2, D1, D2, R1, R2, P1, P2));
    }

    @SuppressWarnings("ResultOfMethodCallIgnored")
    private void saveCalibration() throws IOException {

        final File file = new File("./calibdata");

        if (file.exists()) {
            file.delete();
        }

        file.createNewFile();

        try (final ObjectOutputStream objectOutputStream = new ObjectOutputStream(new FileOutputStream(file))) {
            objectOutputStream.writeObject(ImageUtils.matToJson(CM1));
            objectOutputStream.writeObject(ImageUtils.matToJson(CM2));
            objectOutputStream.writeObject(ImageUtils.matToJson(D1));
            objectOutputStream.writeObject(ImageUtils.matToJson(D2));
            objectOutputStream.writeObject(ImageUtils.matToJson(R1));
            objectOutputStream.writeObject(ImageUtils.matToJson(R2));
            objectOutputStream.writeObject(ImageUtils.matToJson(P1));
            objectOutputStream.writeObject(ImageUtils.matToJson(P2));
        }
    }
}
