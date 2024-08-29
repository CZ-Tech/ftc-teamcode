package org.firstinspires.ftc.teamcode;

import java.io.FileInputStream;
//import java.io.FileOutputStream;
import java.io.IOException;
import java.util.HashMap;
import java.util.Properties;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.HashMap;

public class LoadProperties {
    private String file;

    private String target;
    private Matcher matcher;
    private Pattern pattern;
    private final String pattern_string = "^(\\w+)\\.(\\w+)\\.(\\w+)$";
    public LoadProperties(String file_path, String name) {
        file = file_path;
        target = name;
        pattern = Pattern.compile(pattern_string);
    }

    public HashMap<String, Double> load() throws Throwable {
        Properties prop = new Properties();
        //ArrayList<String[]> myargs = new ArrayList<String[]>();
        HashMap<String, Double> myargs = new HashMap<>();
        try {
            // 加载配置文件
            prop.load(new FileInputStream(file));
            for (String key : prop.stringPropertyNames()) {
                matcher = pattern.matcher(key);
                if (matcher.find()) {
                    myargs.put(matcher.group(2) +"." + matcher.group(3), Double.parseDouble(prop.getProperty(key, "0")));
                }
            }
        } catch (IOException e) {
            throw e.fillInStackTrace();
        }
        return myargs;

    }
}
