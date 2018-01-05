package components;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.util.Base64;

public final class ImageUtils {
    private ImageUtils() {
    }

    public static void draw(final JPanel jPanel, final Mat mat) {
        final MatOfByte mem = new MatOfByte();

        Imgcodecs.imencode(".bmp", mat, mem);

        Image im = null;
        try {
            im = ImageIO.read(new ByteArrayInputStream(mem.toArray()));
        } catch (final IOException e1) {
            e1.printStackTrace();
        }
        final BufferedImage buff = (BufferedImage) im;

        final Graphics g = jPanel.getGraphics();

        if (buff != null) {
            g.drawImage(buff, 0, 0, null);
        }
    }

    public static String matToJson(final Mat mat) {
        final JsonObject obj = new JsonObject();

        if (mat.isContinuous()) {
            final int cols = mat.cols();
            final int rows = mat.rows();
            final int elemSize = (int) mat.elemSize();

            final double[] data = new double[cols * rows * elemSize];

            mat.get(0, 0, data);

            obj.addProperty("rows", mat.rows());
            obj.addProperty("cols", mat.cols());
            obj.addProperty("type", mat.type());

            // We cannot set binary data to a json object, so:
            // Encoding data byte array to Base64.
            final String dataString = new String(Base64.getEncoder().encode(doubleToByteArray(data)));

            obj.addProperty("data", dataString);

            final Gson gson = new Gson();

            return gson.toJson(obj);
        } else {
            System.out.println("Mat not continuous.");
        }
        return "{}";

    }

    public static Mat matFromJson(final String json) {
        final JsonParser parser = new JsonParser();
        final JsonObject JsonObject = parser.parse(json).getAsJsonObject();

        final int rows = JsonObject.get("rows").getAsInt();
        final int cols = JsonObject.get("cols").getAsInt();
        final int type = JsonObject.get("type").getAsInt();

        final String dataString = JsonObject.get("data").getAsString();
        final double[] data = byteToDoubleArray(Base64.getDecoder().decode(dataString.getBytes()));

        final Mat mat = new Mat(rows, cols, type);
        mat.put(0, 0, data);

        return mat;

    }

    public static Mat BufferedImage2Mat(BufferedImage bi) {
        Mat mat = new Mat(bi.getHeight(), bi.getWidth(), CvType.CV_8UC3);
        byte[] data = ((DataBufferByte) bi.getRaster().getDataBuffer()).getData();
        mat.put(0, 0, data);

        return mat; //mat.submat(new Rect(0, 0, img1.width(), img1.height()));
    }


    private static byte[] doubleToByteArray(final double[] doubleArray) {
        final ByteBuffer buf = ByteBuffer.allocate(Double.SIZE / Byte.SIZE * doubleArray.length);
        buf.asDoubleBuffer().put(doubleArray);
        return buf.array();
    }

    private static double[] byteToDoubleArray(final byte[] bytes) {
        final DoubleBuffer buf = ByteBuffer.wrap(bytes).asDoubleBuffer();
        final double[] doubleArray = new double[buf.limit()];
        buf.get(doubleArray);
        return doubleArray;
    }

}
