import components.Corner;
import components.ImageUtils;
import components.Rule;
import org.opencv.calib3d.Calib3d;
import org.opencv.calib3d.StereoBM;
import org.opencv.calib3d.StereoSGBM;
import org.opencv.core.*;
import org.opencv.highgui.Highgui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowEvent;
import java.io.*;
import java.util.concurrent.atomic.AtomicBoolean;

public class Main extends JFrame {

    private static final double FOCUS_LENGTH_PX = 113.3858267717d;
    private static final double BASE_LENGTH_PX = 377.9527559055d;


    private VideoCapture webSourceOne;
    private VideoCapture webSourceTwo;


    private JPanel panel1;
    private JButton startDMButton;
    private JScrollBar scrollBar1;
    private JLabel labelY;
    private JScrollPane scrollPane1;
    private JPanel panel4;
    private JPanel panel2;
    private JScrollBar scrollBar2;
    private JScrollBar scrollBar3;
    private JPanel panel3;
    private JPanel panel5;
    private double maxValue = Double.MIN_VALUE;

    private final Mat img1 = new Mat();
    private final Mat img2 = new Mat();

    private final Mat imgU1 = new Mat();
    private final Mat imgU2 = new Mat();

    private final Mat map1x = new Mat();
    private final Mat map1y = new Mat();
    private final Mat map2x = new Mat();
    private final Mat map2y = new Mat();

    private Mat D1 = new Mat();
    private Mat D2 = new Mat();
    private Mat R1 = new Mat();
    private Mat R2 = new Mat();
    private Mat P1 = new Mat();
    private Mat P2 = new Mat();

    private Mat CM1 = Mat.eye(3, 3, CvType.CV_64F);
    private Mat CM2 = Mat.eye(3, 3, CvType.CV_64F);

    private final AtomicBoolean isDepthRun = new AtomicBoolean();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    @Override
    protected void processWindowEvent(final WindowEvent e) {


        if (e.getID() == WindowEvent.WINDOW_CLOSING) {
            webSourceOne.release();
            webSourceTwo.release();
        }

        super.processWindowEvent(e);
    }

    private Main() {

        webSourceOne = new VideoCapture(0);
        webSourceTwo = new VideoCapture(1);

        $$$setupUI$$$();
        setContentPane(panel1);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        pack();

        if (!loadCalibration()) {
            final Calib calib = new Calib();

            EventQueue.invokeLater(() ->
                    calib.setVisible(true));
        }

        startDMButton.addActionListener(e -> {


            if (isDepthRun.get()) {
                isDepthRun.set(false);
                return;
            }

            webSourceOne.open(0);
            webSourceTwo.open(1);

            webSourceOne.read(img1);

            webSourceTwo.read(img2);

            Imgproc.initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CvType.CV_32FC1, map1x, map1y);
            Imgproc.initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CvType.CV_32FC1, map2x, map2y);


            final Runnable depthRunnable = () -> {
                while (isDepthRun.get()) {

                    webSourceOne.read(img1);

                    webSourceTwo.read(img2);

                    Imgproc.cvtColor(img1, img1, Imgproc.COLOR_BGR2GRAY);

                    Imgproc.cvtColor(img2, img2, Imgproc.COLOR_BGR2GRAY);

                    undistortAndRemap();

                    ImageUtils.draw(panel5, imgU1);

                    ImageUtils.draw(panel3, imgU2);

                    final StereoBM stereoSGBM = StereoBM.create();

                    final Mat disparity = new Mat();

                    stereoSGBM.compute(imgU1, imgU2, disparity);

                    final Mat normalized = new Mat();

                    Core.normalize(disparity, normalized, 0, 255, Core.NORM_MINMAX);

                    final double[][][] distances = new double[normalized.rows()][normalized.cols()][1];

                    for (int i = 0, width = normalized.cols(); i < width; i++) {
                        for (int j = 0, height = normalized.rows(); j < height; j++) {

                            final double distancePx = getDistancePx(normalized.get(j, i)[0]);

                            maxValue = distancePx > maxValue ? maxValue = distancePx : maxValue;

                            distances[j][i][0] = distancePx;
                        }
                    }

                    final int index = (int) ((float) normalized.rows() / 100f * (float) scrollBar1.getValue());

                    labelY.setText("Slice index " + index);

                    drawDistances(panel2, distances[index]);

                    ImageUtils.draw(panel4, normalized);

                }
            };

            if (!isDepthRun.get()) {

                isDepthRun.set(true);

                new Thread(depthRunnable).start();

            }

        });

    }

    private void undistortAndRemap() {
        Imgproc.remap(img1, imgU1, map1x, map1y, Imgproc.INTER_LINEAR);
        Imgproc.remap(img2, imgU2, map2x, map2y, Imgproc.INTER_LINEAR);
    }

    private boolean loadCalibration() {

        final File file = new File("./calibdata");

        if (!file.exists()) {
            return false;
        }
        try (final ObjectInputStream objectInputStream = new ObjectInputStream(new FileInputStream(file))) {
            try {
                CM1 = ImageUtils.matFromJson((String) objectInputStream.readObject());
                CM2 = ImageUtils.matFromJson((String) objectInputStream.readObject());
                D1 = ImageUtils.matFromJson((String) objectInputStream.readObject());
                D2 = ImageUtils.matFromJson((String) objectInputStream.readObject());
                R1 = ImageUtils.matFromJson((String) objectInputStream.readObject());
                R2 = ImageUtils.matFromJson((String) objectInputStream.readObject());
                P1 = ImageUtils.matFromJson((String) objectInputStream.readObject());
                P2 = ImageUtils.matFromJson((String) objectInputStream.readObject());
            } catch (final IOException | ClassNotFoundException e) {
                return false;
            }

            return true;
        } catch (final IOException e) {
            return false;
        }
    }

    private void createUIComponents() {
        panel4 = new JPanel();
        panel2 = new JPanel();
        panel3 = new JPanel();
        panel5 = new JPanel();

        final Rule columnView = new Rule(Rule.HORIZONTAL, true);

        columnView.setPreferredWidth(50);

        final Rule rowView = new Rule(Rule.VERTICAL, true);

        rowView.setPreferredHeight(50);

        scrollPane1 = new JScrollPane();

        scrollPane1.setColumnHeaderView(columnView);
        scrollPane1.setRowHeaderView(rowView);

        //Create the corners.
        final JPanel buttonCorner = new JPanel(); //use FlowLayout
        final JToggleButton isMetric = new JToggleButton("cm", true);
        isMetric.setFont(new Font("SansSerif", Font.PLAIN, 11));
        isMetric.setMargin(new Insets(2, 2, 2, 2));
        isMetric.addItemListener(e -> System.out.println("Corner button"));
        buttonCorner.add(isMetric);

        scrollPane1.setCorner(JScrollPane.UPPER_LEFT_CORNER,
                buttonCorner);
        scrollPane1.setCorner(JScrollPane.LOWER_LEFT_CORNER,
                new Corner());
        scrollPane1.setCorner(JScrollPane.UPPER_RIGHT_CORNER,
                new Corner());
    }

    /**
     * @param args the command line arguments
     */
    public static void main(final String[] args) {

        final Main main = new Main();

        try {
            EventQueue.invokeLater(() ->
                    main.setVisible(true));
        } finally {
            if (null != main.webSourceOne && main.webSourceOne.isOpened()) {
                main.webSourceOne.release();
            }

            if (null != main.webSourceTwo && main.webSourceTwo.isOpened()) {
                main.webSourceTwo.release();
            }
        }
    }

    private double getDistancePx(final double disparity) {
        if (disparity == 0) {
            return 0;
        }

        return FOCUS_LENGTH_PX * BASE_LENGTH_PX / disparity;
    }

    private void drawDistances(final JPanel jPanel, final double[][] mat) {

        jPanel.removeAll();

        jPanel.updateUI();

        jPanel.setPreferredSize(new Dimension(mat.length, (int) maxValue));

        for (int j = 0, cols = mat.length; j < cols; j++) {

            final double original = mat[j][0];

            final int point = (int) original;

            jPanel.getGraphics().drawLine(j, point, j, point);
        }

        maxValue = Double.MIN_VALUE;
    }


    /**
     * Method generated by IntelliJ IDEA GUI Designer
     * >>> IMPORTANT!! <<<
     * DO NOT edit this method OR call it in your code!
     *
     * @noinspection ALL
     */
    private void $$$setupUI$$$() {
        createUIComponents();
        panel1 = new JPanel();
        panel1.setLayout(new com.intellij.uiDesigner.core.GridLayoutManager(3, 4, new Insets(0, 0, 0, 0), -1, -1));
        panel1.add(panel4, new com.intellij.uiDesigner.core.GridConstraints(0, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, new Dimension(384, 288), new Dimension(384, 288), new Dimension(384, 288), 0, false));
        startDMButton = new JButton();
        startDMButton.setText("Start DM");
        panel1.add(startDMButton, new com.intellij.uiDesigner.core.GridConstraints(2, 0, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_HORIZONTAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        scrollBar1 = new JScrollBar();
        scrollBar1.setMaximum(109);
        scrollBar1.setOrientation(1);
        scrollBar1.setValue(99);
        scrollBar1.setVisible(true);
        scrollBar1.setVisibleAmount(10);
        panel1.add(scrollBar1, new com.intellij.uiDesigner.core.GridConstraints(0, 3, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_VERTICAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
        labelY = new JLabel();
        labelY.setText("Label");
        panel1.add(labelY, new com.intellij.uiDesigner.core.GridConstraints(2, 1, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_NONE, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, null, null, null, 0, false));
        scrollPane1.setBackground(new Color(-1311233));
        scrollPane1.setHorizontalScrollBarPolicy(30);
        scrollPane1.setVerticalScrollBarPolicy(20);
        panel1.add(scrollPane1, new com.intellij.uiDesigner.core.GridConstraints(0, 1, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_BOTH, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_CAN_SHRINK | com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, new Dimension(640, 480), null, null, 0, false));
        panel2.setBackground(new Color(-1442305));
        scrollPane1.setViewportView(panel2);
        scrollBar2 = new JScrollBar();
        scrollBar2.setOrientation(0);
        panel1.add(scrollBar2, new com.intellij.uiDesigner.core.GridConstraints(1, 1, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_VERTICAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
        scrollBar3 = new JScrollBar();
        panel1.add(scrollBar3, new com.intellij.uiDesigner.core.GridConstraints(0, 2, 1, 1, com.intellij.uiDesigner.core.GridConstraints.ANCHOR_CENTER, com.intellij.uiDesigner.core.GridConstraints.FILL_VERTICAL, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_FIXED, com.intellij.uiDesigner.core.GridConstraints.SIZEPOLICY_WANT_GROW, null, null, null, 0, false));
        scrollPane1.setHorizontalScrollBar(scrollBar2);
        scrollPane1.setVerticalScrollBar(scrollBar3);
    }

    /**
     * @noinspection ALL
     */
    public JComponent $$$getRootComponent$$$() {
        return panel1;
    }
}
