package components;

import org.opencv.core.Mat;

public final class MatUtils {
    private MatUtils() {
    }

    public static String getCalibInfo(final Mat CM1,
                                final Mat CM2,
                                final Mat D1,
                                final Mat D2,
                                final Mat R1,
                                final Mat R2,
                                final Mat P1,
                                final Mat P2) {
        return "Camera matrix 1: \n\n" + MatUtils.getMat2fForPrint(CM1) + "\n\n"
                + "Camera matrix 2: \n\n" + MatUtils.getMat2fForPrint(CM2) + "\n\n"
                + "Distortion coefficients 1: " + "\n\n" + MatUtils.getMat2fForPrint(D1) + "\n\n"
                + "Distortion coefficients 2: " + "\n\n" + MatUtils.getMat2fForPrint(D2) + "\n\n"
                + "Rotation matrix 1" + "\n\n" + MatUtils.getMat2fForPrint(R1) + "\n\n"
                + "Rotation matrix 2" + "\n\n" + MatUtils.getMat2fForPrint(R2) + "\n\n"
                + "Projection matrix 1: " + "\n\n" + MatUtils.getMat2fForPrint(P1) + "\n\n"
                + "Projection matrix 2: " + "\n\n" + MatUtils.getMat2fForPrint(P2);
    }

    private static String getMat2fForPrint(final Mat mat) {

        final StringBuilder stringBuilder = new StringBuilder();

        for (int i = 0; i < mat.rows(); i++) {
            for (int j = 0; j < mat.cols(); j++) {

                if (j != 0 && j < mat.cols() - 1) {
                    stringBuilder.append(", ");
                } else if (j == 0) {
                    stringBuilder.append("[ ");
                } else if (j == mat.cols() - 1) {
                    stringBuilder.append(" ]");
                }

                stringBuilder.append(mat.get(i, j)[0]);
            }

            if (i < mat.rows() - 1) {
                stringBuilder.append("\n");
            }
        }

        return stringBuilder.toString();
    }
}
