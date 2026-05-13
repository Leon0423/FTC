package org.firstinspires.ftc.teamcode.swerve;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class SwerveTuningStorage {

    private static final String FILE_NAME = "swerve_tuning.txt";

    public static void save(SwerveTuningData data) {
        File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);

        String text =
                data.maxTranslation + "\n" +
                        data.maxRotation + "\n" +
                        data.steerKP + "\n" +
                        data.steerKD + "\n" +
                        data.steerMaxPower + "\n" +
                        data.steerToleranceDeg + "\n";

        ReadWriteFile.writeFile(file, text);
    }

    public static SwerveTuningData load() {
        SwerveTuningData data = new SwerveTuningData();

        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) return data;

            String text = ReadWriteFile.readFile(file).trim();
            if (text.isEmpty()) return data;

            String[] lines = text.split("\\R");
            if (lines.length >= 6) {
                data.maxTranslation = Double.parseDouble(lines[0].trim());
                data.maxRotation = Double.parseDouble(lines[1].trim());
                data.steerKP = Double.parseDouble(lines[2].trim());
                data.steerKD = Double.parseDouble(lines[3].trim());
                data.steerMaxPower = Double.parseDouble(lines[4].trim());
                data.steerToleranceDeg = Double.parseDouble(lines[5].trim());
            }
        } catch (Exception ignored) {
        }

        return data;
    }
}