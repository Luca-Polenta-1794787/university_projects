package com.example.myapplication

import android.app.Activity
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.view.MotionEvent
import android.view.View
import androidx.core.content.ContextCompat

class MainActivity : Activity(), SensorEventListener {
    lateinit var myView : MyView

    lateinit var mSensorManager : SensorManager
    //accelerometer stuff
    private var acc_x = 0f
    var acc_y = 0f
    var acc_z = 0f
    //comment
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        mSensorManager = getSystemService(Context.SENSOR_SERVICE) as SensorManager
        mSensorManager.registerListener(
            this,
            mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
            SensorManager.SENSOR_DELAY_NORMAL
        )


        myView = MyView(this)
        setContentView(myView)

    }

    override fun onSensorChanged(event: SensorEvent?) {


        acc_x = event!!.values[0]
        acc_y = event!!.values[1]
        acc_z = event!!.values[2]

        myView.setAcc(acc_x, acc_y, acc_z)

    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {

    }

}