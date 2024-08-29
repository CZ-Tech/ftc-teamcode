package org.firstinspires.ftc.teamcode.utlity;

import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.util.Properties;

public class FileUtil {
    public String read(String pathname){
        return ReadWriteFile.readFile(new File(pathname));
    }
    public void write(String pathname,String fileContents){
        ReadWriteFile.writeFile(new File(pathname),fileContents);
        Properties prop = new Properties();

    }
}
