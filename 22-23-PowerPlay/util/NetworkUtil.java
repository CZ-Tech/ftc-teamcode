package org.firstinspires.ftc.teamcode.util;

import android.util.Pair;

import com.google.gson.JsonObject;

import javax.net.ssl.HttpsURLConnection;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;

public class NetworkUtil {

    public static String getDataWithJson(String url, JsonObject json) {
        return getDataWithJson(url, json.toString());
    }

    public static String getDataWithJson(String url, String json) {
        List<Pair<String, String>> header = new ArrayList<>();
        header.add(new Pair<>("Content-Type", "application/json"));
        header.add(new Pair<>("Accept", "application/json"));
        return getData(url, Method.POST, header, json);
    }

    public static String getDataWithHeader(String url, List<Pair<String, String>> header) {
        return getData(url, Method.GET, header, "");
    }

    public static String getData(String url) {
        return getData(url, Method.GET, new ArrayList<>(), null);
    }

    public static String getData(String url, Method method, List<Pair<String, String>> header, String data) {
        HttpsURLConnection con = null;
        try {
            con = (HttpsURLConnection) new URL(url).openConnection();
            con.setRequestMethod(method.text);
            con.setDoInput(true);
            con.setDoOutput(true);
            con.setUseCaches(false);
            if (header.size() > 0)
                for (Pair<String, String> p : header)
                    con.setRequestProperty(p.first, p.second);
            if (method == Method.POST) {
                OutputStreamWriter osw = new OutputStreamWriter(con.getOutputStream(), StandardCharsets.UTF_8);
                osw.write(data);
                osw.flush();
                osw.close();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        if (con != null)
            con.disconnect();
        else
            return "";

        StringBuilder builder = new StringBuilder();
        try {
            BufferedReader br = new BufferedReader(new InputStreamReader(con.getInputStream(), StandardCharsets.UTF_8));
            String temp;
            while (((temp = br.readLine())) != null)
                builder.append(temp).append("\n");
        } catch (Exception e) {
            e.printStackTrace();
        }
        return builder.toString();
    }

    private enum Method {
        POST("POST"), GET("GET");
        private final String text;

        Method(String text) {
            this.text = text;
        }
    }

    public interface Callback<T> {
        boolean execute(T data);
    }
}