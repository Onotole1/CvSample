import components.ImageUtils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import javax.swing.*;
import java.awt.event.WindowEvent;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class Calib extends JFrame {

    private static final Size PATTERN_SIZE = new Size(7, 7);

    private static final double SQUARE_SIZE = 15;

    private final int mCornersSize = (int) (PATTERN_SIZE.width * PATTERN_SIZE.height);

    private JLabel label1;
    private JButton finishButton;
    private JPanel panel1;
    private JPanel panel2;
    private JPanel panel3;

    private final Mat img1 = new Mat();
    private final Mat img2 = new Mat();


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

    private VideoCapture webSourceOne;
    private VideoCapture webSourceTwo;

    private final AtomicBoolean isCalibrationRun = new AtomicBoolean();

    public Calib() {

        setContentPane(panel3);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        pack();

        final Runnable depthRunnable = () -> {

            while (isCalibrationRun.get()) {

                webSourceOne.read(img1);

                webSourceTwo.read(img2);

                Imgproc.cvtColor(img1, gray1, Imgproc.COLOR_BGR2GRAY);

                Imgproc.cvtColor(img2, gray2, Imgproc.COLOR_BGR2GRAY);

                final MatOfPoint2f corners1 = new MatOfPoint2f();

                final MatOfPoint2f corners2 = new MatOfPoint2f();


                final boolean found1 = Calib3d.findChessboardCorners(img1, PATTERN_SIZE, corners1,
                        Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_FILTER_QUADS);
                final boolean found2 = Calib3d.findChessboardCorners(img2, PATTERN_SIZE, corners2,
                        Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_FILTER_QUADS);

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

                    imagePoints1.add(corners1);
                    imagePoints2.add(corners2);


                    final Mat zeros = Mat.zeros(mCornersSize, 1, CvType.CV_32FC3);

                    calcBoardCornerPositions(zeros);
                    objectPoints.add(zeros);

                    label1.setText("Calibrate images count: " + imagePoints1.size());

                    try {
                        Thread.sleep(1000);
                    } catch (final InterruptedException e) {
                        //do nothing
                    }
                }

            }

            webSourceOne.release();
            webSourceTwo.release();
        };


        webSourceOne = new VideoCapture(0);

        webSourceTwo = new VideoCapture(1);

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
                calibrate();
                try {
                    saveCalibration();
                    dispatchEvent(new WindowEvent(this, WindowEvent.WINDOW_CLOSING));
                } catch (final IOException exception) {
                    //do nothing
                }
            }
        });
    }

    private void calibrate() {

        System.out.println(objectPoints.size() + " " + imagePoints1.size() + " " + imagePoints2.size());

        Calib3d.stereoCalibrate(objectPoints, imagePoints1, imagePoints2, CM1, D1, CM2, D2, img1.size(), R, T, E, F, Calib3d.CALIB_SAME_FOCAL_LENGTH | Calib3d.CALIB_ZERO_TANGENT_DIST, new TermCriteria(TermCriteria.MAX_ITER + TermCriteria.EPS, 100, 1e-5));

        Calib3d.stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
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
