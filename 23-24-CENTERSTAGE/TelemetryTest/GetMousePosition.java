package org.firstinspires.ftc.teamcode.TelemetryTest;

import android.content.Context;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;

import androidx.appcompat.app.AppCompatActivity;

public class GetMousePosition extends AppCompatActivity {

    MyView myView;
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        // 创建自定义视图
        this.myView = new MyView(this);
        // 设置为ContentView
        setContentView(myView);
    }


    public class MyView extends View {

        float[] position = new float[2];

        public MyView(Context context) {
            super(context);
        }

        @Override
        public boolean onTouchEvent(MotionEvent event) {
            switch (event.getAction()) {
                case MotionEvent.ACTION_DOWN:
                case MotionEvent.ACTION_MOVE:
                case MotionEvent.ACTION_UP:
                    // 处理鼠标事件
                    handleMouseEvent(event);
                    break;
            }
            return true;
        }

        private void handleMouseEvent(MotionEvent event) {
            // 获取鼠标的位置坐标
            float x = event.getX();
            float y = event.getY();
            // 显示坐标信息
            this.position[0] = x;
            this.position[1] = y;
        }

        public float[] getPosition() {
            return this.position;
        }
    }
}