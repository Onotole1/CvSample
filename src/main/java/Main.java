import components.*;
import de.humatic.dsj.DSCapture;
import de.humatic.dsj.DSFilterInfo;
import de.humatic.dsj.DSFiltergraph;
import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;
import org.opencv.calib3d.StereoBM;
import org.opencv.calib3d.StereoSGBM;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import javax.swing.*;
import java.awt.*;
import java.awt.event.WindowEvent;
import java.io.*;
import java.util.Enumeration;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class Main extends JFrame implements SerialPortEventListener {

    private static final double FOCUS_LENGTH_PX = 113.3858267717d;
    private static final double BASE_LENGTH_PX = 377.9527559055d;

    private SerialPort serialPort;

    private static final String PORT_NAMES[] = {
            "COM4", // Windows
    };
    private BufferedReader input;
    private static final int TIME_OUT = 2000;
    private static final int DATA_RATE = 9600;

    private DSCapture cam1graph;
    private DSCapture cam2graph;

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
    private JComboBox comboBox1;
    private JSpinner numDisparitiesSpinner;
    private JSpinner blockSizeSpinner;
    private JSpinner windowSizeSpinner;
    private JSpinner minDispSpinner;
    private JSpinner disp12MaxDiffSpinner;
    private JSpinner preFilterCapSpinner;
    private JSpinner uniquenessRatioSpinner;
    private JSpinner speckleWindowSizeSpinner;
    private JSpinner speckleRangeSpinner;
    private JButton setDefaultsButton;
    private JLabel windowSizeLabel;
    private JLabel minDispLabel;
    private JLabel disp12MaxDiffLabel;
    private JLabel preFilterCapLabel;
    private JLabel uniquenessRatioLabel;
    private JLabel speckleWindowSizeLabel;
    private JLabel speckleRangeLabel;
    private JComboBox<String> comboBoxRotation2;
    private JComboBox<String> comboBoxRotation1;
    private double maxValue = Double.MIN_VALUE;

    private Mat img1;
    private Mat img2;

    private Mat imgU1 = new Mat();
    private Mat imgU2 = new Mat();

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

    private final AtomicInteger selectedRotation1 = new AtomicInteger();
    private final AtomicInteger selectedRotation2 = new AtomicInteger();

    private final AtomicBoolean isDepthRun = new AtomicBoolean();

    private final PreferencesProvider preferencesProvider = new PreferencesProvider();

    static {
        final String opencvpath = System.getProperty("user.dir") + "\\libs\\";
        System.load(opencvpath + Core.NATIVE_LIBRARY_NAME + ".dll");
    }

    @Override
    protected void processWindowEvent(final WindowEvent e) {


        if (e.getID() == WindowEvent.WINDOW_CLOSING) {
            cam1graph.stop();
            cam2graph.stop();

            if (serialPort != null) {
                serialPort.removeEventListener();
                serialPort.close();

                if (null != input) {
                    try {
                        input.close();
                    } catch (final IOException e1) {
                        //do nothing
                    }
                }
            }
        }

        super.processWindowEvent(e);
    }

    Main() {

        $$$setupUI$$$();
        initViews();
        initializeSerial();
        setContentPane(panel1);
        setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        pack();

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

        startDMButton.addActionListener(e -> {


            if (isDepthRun.get()) {
                isDepthRun.set(false);
                return;
            }


            final DSFilterInfo[][] info = DSCapture.queryDevices();
            final DSFilterInfo easyCap1 = info[0][0];
            final DSFilterInfo easyCap2 = info[0][1];

            cam1graph = new DSCapture(DSFiltergraph.CAPTURE, easyCap1,
                    false, DSFilterInfo.doNotRender(), null);


            cam1graph.setVisible(true);
            cam1graph.setPreview();

            cam2graph = new DSCapture(DSFiltergraph.CAPTURE, easyCap2,
                    false, DSFilterInfo.doNotRender(), null);


            cam2graph.setVisible(true);
            cam2graph.setPreview();

            img1 = cam1graph != null ?
                    ImageUtils.bufferedImage2Mat(cam1graph.getImage()) : new Mat();

            img2 = cam1graph != null ?
                    ImageUtils.bufferedImage2Mat(cam2graph.getImage()) : new Mat();

            Imgproc.initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CvType.CV_32FC1, map1x, map1y);
            Imgproc.initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CvType.CV_32FC1, map2x, map2y);


            final Runnable depthRunnable = () -> {
                while (isDepthRun.get()) {

                    img1 = ImageUtils.bufferedImage2Mat(cam1graph.getImage());

                    img2 = ImageUtils.bufferedImage2Mat(cam2graph.getImage());

                    assert img1 != null && img2 != null;

                    final int rotation1 = selectedRotation1.get();
                    if (rotation1 != Constants.NO_ROTATION_VALUE) {
                        Core.flip(img1, img1, rotation1);
                    }

                    final int rotation2 = selectedRotation2.get();
                    if (rotation2 != Constants.NO_ROTATION_VALUE) {
                        Core.flip(img2, img2, rotation2);
                    }


                    Imgproc.cvtColor(img1, img1, Imgproc.COLOR_BGR2GRAY);

                    Imgproc.cvtColor(img2, img2, Imgproc.COLOR_BGR2GRAY);

                    //undistortAndRemap();

                    imgU1 = img1;
                    imgU2 = img2;

                    ImageUtils.draw(panel5, imgU1);

                    ImageUtils.draw(panel3, imgU2);

                    final Mat disparity = new Mat();

                    if (comboBox1.getSelectedIndex() == 0) {
                        final StereoBM stereoSGBM = StereoBM.create((Integer) numDisparitiesSpinner.getValue(), (Integer) blockSizeSpinner.getValue());

                        stereoSGBM.compute(imgU1, imgU2, disparity);

                    } else if (comboBox1.getSelectedIndex() == 1) {

                        final int windowSize = (Integer) windowSizeSpinner.getValue();
                        final int minDisp = (Integer) minDispSpinner.getValue();
                        final int numDisp = (Integer) numDisparitiesSpinner.getValue();
                        final int blockSize = (Integer) blockSizeSpinner.getValue();
                        final int P1 = (int) Math.pow(8 * 3 * windowSize, 2);
                        final int P2 = (int) Math.pow(32 * 3 * windowSize, 2);
                        final int disp12MaxDiff = (Integer) disp12MaxDiffSpinner.getValue();
                        final int preFilterCap = (Integer) preFilterCapSpinner.getValue();
                        final int uniquenessRatio = (Integer) uniquenessRatioSpinner.getValue();
                        final int speckleWindowSize = (Integer) speckleWindowSizeSpinner.getValue();
                        final int speckleRange = (Integer) speckleRangeSpinner.getValue();

                        final StereoSGBM stereoSGBM = StereoSGBM.create(minDisp, numDisp, blockSize, P1, P2, disp12MaxDiff, preFilterCap, uniquenessRatio, speckleWindowSize, speckleRange, StereoSGBM.MODE_HH);

                        stereoSGBM.compute(imgU1, imgU2, disparity);
                    }

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
        comboBox1 = new JComboBox();

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

    private void initViews() {
        numDisparitiesSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getNumDisparities(), 16, Integer.MAX_VALUE, 16));
        numDisparitiesSpinner.addChangeListener(e -> {
                    if ((Integer) numDisparitiesSpinner.getValue() % 16 == 0) {
                        preferencesProvider.saveNumDisparities((Integer) numDisparitiesSpinner.getValue());
                    } else {
                        numDisparitiesSpinner.setValue(numDisparitiesSpinner.getPreviousValue());
                    }
                }
        );

        blockSizeSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getBlockSize(), 5, 255, 2));
        blockSizeSpinner.addChangeListener(e -> preferencesProvider.saveBlockSize((Integer) blockSizeSpinner.getValue()));

        speckleRangeSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getSpeckleRange(), 1, Integer.MAX_VALUE, 1));
        speckleRangeSpinner.addChangeListener(e -> preferencesProvider.saveSpeckleRange((Integer) speckleRangeSpinner.getValue()));

        speckleWindowSizeSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getSpeckleWindowSize(), 1, Integer.MAX_VALUE, 1));
        speckleWindowSizeSpinner.addChangeListener(e -> preferencesProvider.saveSpeckleWindowSize((Integer) speckleWindowSizeSpinner.getValue()));

        preFilterCapSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getPreFiltered(), 0, Integer.MAX_VALUE, 1));
        preFilterCapSpinner.addChangeListener(e -> preferencesProvider.savePreFiltered((Integer) preFilterCapSpinner.getValue()));

        disp12MaxDiffSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getDisp12MaxDisp(), 1, Integer.MAX_VALUE, 1));
        disp12MaxDiffSpinner.addChangeListener(e -> preferencesProvider.saveDisp12MaxDisp((Integer) disp12MaxDiffSpinner.getValue()));

        windowSizeSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getWindowSize(), 1, Integer.MAX_VALUE, 1));
        windowSizeSpinner.addChangeListener(e -> preferencesProvider.saveWindowSize((Integer) windowSizeSpinner.getValue()));

        minDispSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getMinDisparities(), 1, Integer.MAX_VALUE, 1));
        minDispSpinner.addChangeListener(e -> preferencesProvider.saveMinDisparities((Integer) minDispSpinner.getValue()));

        uniquenessRatioSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getUniquenessRatio(), 1, Integer.MAX_VALUE, 1));
        uniquenessRatioSpinner.addChangeListener(e -> preferencesProvider.saveUniquenessRatio((Integer) uniquenessRatioSpinner.getValue()));

        setDefaultsButton.addActionListener(e -> {
            preferencesProvider.setDefaults();

            numDisparitiesSpinner.setValue(preferencesProvider.getNumDisparities());
            blockSizeSpinner.setValue(preferencesProvider.getBlockSize());
            speckleRangeSpinner.setValue(preferencesProvider.getSpeckleRange());
            speckleWindowSizeSpinner.setValue(preferencesProvider.getWindowSize());
            preFilterCapSpinner.setValue(preferencesProvider.getPreFiltered());
            disp12MaxDiffSpinner.setValue(preferencesProvider.getDisp12MaxDisp());
            windowSizeSpinner.setValue(preferencesProvider.getWindowSize());
            minDispSpinner.setValue(preferencesProvider.getMinDisparities());
            uniquenessRatioSpinner.setValue(preferencesProvider.getUniquenessRatio());
        });

        comboBox1.addActionListener(e -> {

            final int selectedIndex = comboBox1.getSelectedIndex();

            if (selectedIndex == 0) {
                windowSizeSpinner.setVisible(false);
                minDispSpinner.setVisible(false);
                disp12MaxDiffSpinner.setVisible(false);
                preFilterCapSpinner.setVisible(false);
                uniquenessRatioSpinner.setVisible(false);
                speckleWindowSizeSpinner.setVisible(false);
                speckleRangeSpinner.setVisible(false);

                disp12MaxDiffLabel.setVisible(false);
                minDispLabel.setVisible(false);
                preFilterCapLabel.setVisible(false);
                speckleRangeLabel.setVisible(false);
                uniquenessRatioLabel.setVisible(false);
                windowSizeLabel.setVisible(false);
                speckleWindowSizeLabel.setVisible(false);

                final int preferencesProviderBlockSize = preferencesProvider.getBlockSize();

                if (preferencesProviderBlockSize % 2 == 0 || preferencesProviderBlockSize < 5 || preferencesProviderBlockSize > 255) {

                    if (preferencesProviderBlockSize < 5) {
                        preferencesProvider.saveBlockSize(5);
                    } else if (preferencesProviderBlockSize > 255) {
                        preferencesProvider.saveBlockSize(255);
                    } else {
                        int newValue = preferencesProviderBlockSize - 1;

                        if (newValue < 5) {
                            newValue += 2;
                        }
                        preferencesProvider.saveBlockSize(newValue);
                    }
                }

                blockSizeSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getBlockSize(), 5, 255, 2));

            } else if (selectedIndex == 1) {
                windowSizeSpinner.setVisible(true);
                minDispSpinner.setVisible(true);
                disp12MaxDiffSpinner.setVisible(true);
                preFilterCapSpinner.setVisible(true);
                uniquenessRatioSpinner.setVisible(true);
                speckleWindowSizeSpinner.setVisible(true);
                speckleRangeSpinner.setVisible(true);

                disp12MaxDiffLabel.setVisible(true);
                minDispLabel.setVisible(true);
                preFilterCapLabel.setVisible(true);
                speckleRangeLabel.setVisible(true);
                uniquenessRatioLabel.setVisible(true);
                windowSizeLabel.setVisible(true);
                speckleWindowSizeLabel.setVisible(true);

                blockSizeSpinner.setModel(new SpinnerNumberModel(preferencesProvider.getBlockSize(), 1, Integer.MAX_VALUE, 1));
            }
        });
    }

    /**
     * @param args the command line arguments
     */
    public static void main(final String[] args) {

        final Main main = new Main();

        if (!main.loadCalibration()) {
            final Calib calib = new Calib();

            EventQueue.invokeLater(() ->
                    calib.setVisible(true));
        } else {

            EventQueue.invokeLater(() ->
                    main.setVisible(true));
        }
    }

    private void initializeSerial() {
        CommPortIdentifier portId = null;
        final Enumeration portEnum = CommPortIdentifier.getPortIdentifiers();

        //First, Find an instance of serial port as set in PORT_NAMES.
        while (portEnum.hasMoreElements()) {
            final CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            for (final String portName : PORT_NAMES) {
                if (currPortId.getName().equals(portName)) {
                    portId = currPortId;
                    break;
                }
            }
        }
        if (portId == null) {
            System.out.println("Could not find COM port.");
            return;
        }

        try {
            serialPort = (SerialPort) portId.open(this.getClass().getName(),
                    TIME_OUT);
            serialPort.setSerialPortParams(DATA_RATE,
                    SerialPort.DATABITS_8,
                    SerialPort.STOPBITS_1,
                    SerialPort.PARITY_NONE);

            // open the streams
            input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));

            serialPort.addEventListener(this);
            serialPort.notifyOnDataAvailable(true);
        } catch (final Exception e) {
            System.err.println(e.toString());
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

    @Override
    public synchronized void serialEvent(final SerialPortEvent oEvent) {
        if (oEvent.getEventType() == SerialPortEvent.DATA_AVAILABLE) {
            try {
                final String inputLine;
                if (input.ready()) {
                    inputLine = input.readLine();
                    System.out.println(inputLine);
                }

            } catch (final Exception e) {
                System.err.println(e.toString());
            }
        }
    }
}
