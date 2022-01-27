package com.example.myapplication

import android.content.ContentValues.TAG
import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.util.Log
import android.view.MotionEvent
import android.view.View
import com.example.myapplication.modelsForServerCommunications.GamePosition
import com.google.firebase.database.*
import com.google.firebase.database.DataSnapshot

class MyView(context: Context): View(context), View.OnTouchListener {

    var acc_x = 0f
    var acc_y = 0f
    var acc_z = 0f
    var cannon_pos = 0f
    var enemy_cannon_pos = 0f
    var enemy_x = 0f
    val cannon_rad = 50f

    var touchX = 0f
    var touchY = 0f
    val textPainter = Paint().apply {
        textSize = 50f
        color = Color.WHITE
        //color = Color.parseColor("#FF0000")
    }
    var ballX = 0f
    var ballY = 0f
    var firing = false
    var vx = 1000f
    var vy = 0f
    var dt = 0L
    var past = 0L

    val mass = 1f
    val g = 0
    val radius = 20f

    val database = FirebaseDatabase.getInstance("https://cannon-805e2-default-rtdb.firebaseio.com/");
    var myRef = database.getReference("FightTest");
    val userId = "0"

    private val TAG = "FIREBASE"


    //execute at class init
    init {
        val gp = GamePosition(userId, "0", "0", null)
        myRef.child("GamePosition_"+userId).setValue(gp)
        setOnTouchListener(this)

        // Read from the database for LAT
        myRef.child("GamePosition_"+userId).child("lat").addValueEventListener(object: ValueEventListener {
            override fun onDataChange(dataSnapshot: DataSnapshot) {
                logAndSaveChanges(dataSnapshot)
            }

            override fun onCancelled(databaseError: DatabaseError) {
                // Getting Post failed, log a message
                Log.w(TAG, "loadGamePosition:onCancelled", databaseError.toException())
            }
        })

        // Read from the database for LON
        myRef.child("GamePosition_"+userId).child("lon").addValueEventListener(object: ValueEventListener {
            override fun onDataChange(dataSnapshot: DataSnapshot) {
                logAndSaveChanges(dataSnapshot)
            }

            override fun onCancelled(databaseError: DatabaseError) {
                // Getting Post failed, log a message
                Log.w(TAG, "loadGamePosition:onCancelled", databaseError.toException())
            }

        })
    }

    private fun logAndSaveChanges(dataSnapshot: DataSnapshot) {
        val nomeChiave = dataSnapshot.key
        val datoChiave = dataSnapshot.value
        Log.d(TAG, "Value " + nomeChiave + " is: " + datoChiave)
    }


    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        val now = System.currentTimeMillis()
        dt = (now - past)
        past = now

        canvas.drawRGB(255, 0 , 0)
        //canvas.drawLine(0f, canvas.height.toFloat(), touchX, touchY, textPainter)
        //canvas.drawText("x: " + touchX + " y: " + touchY, 0f, 100f, textPainter)

        //canvas.drawText("by: " + ballY + " cp: " + cannon_pos + " bx: " + ballX, 0f, 100f, textPainter)
        if(firing) {
            canvas.drawCircle(ballX, ballY, radius, textPainter)
            ballX = (vx * dt) / 1000 + ballX
            ballY = (vy * dt) / 1000 + ballY
            //vy = vy + (g*dt).toFloat()
            if(ballX > canvas.width) {
                firing = false
                //vy = 1000f
            }
        }
        drawCannon(canvas)
        invalidate()
    }

    fun drawCannon(canvas: Canvas){
        cannon_pos = ((canvas.height / 9.81) * acc_x ).toFloat()
        canvas.drawCircle(0f, cannon_pos, cannon_rad, textPainter)
        canvas.drawLine(0f,  cannon_pos, touchX, touchY, textPainter)
        enemy_cannon_pos = ((canvas.height / 9.81) * acc_x ).toFloat()
        canvas.drawCircle(canvas.width.toFloat(), cannon_pos, cannon_rad, textPainter)
    }

    override fun onTouch(v: View?, event: MotionEvent?): Boolean {

        when(event?.action){
            MotionEvent.ACTION_DOWN, MotionEvent.ACTION_MOVE -> {
                touchX = event.x
                touchY = event.y
                firing = false

                myRef.child("GamePosition_"+userId).child("lat").setValue(touchX)
                myRef.child("GamePosition_"+userId).child("lon").setValue(touchY)

                invalidate()
            }

            MotionEvent.ACTION_UP -> {
                ballX = event.x
                ballY = event.y
                firing = true
                vy = ((ballY - cannon_pos) / (ballX))*1000
                invalidate()

            }

        }

        return true
    }

    fun setAcc(x : Float, y : Float, z : Float) {

        if(x < 0)
            this.acc_x = 0f
        else if(x > 0 && z < 0f)       //useless
            this.acc_x = 9.81f
        else
            this.acc_x = x

        acc_y = y
        acc_z = z

        invalidate()

    }

    /*
    //LISTENER PRIMA PIÙ BASIC
            /*
            override fun onDataChange(snapshot: DataSnapshot) {
                // This method is called once with the initial value and again
                // whenever data at this location is updated.
                val value = snapshot.getValue<String>()
                Log.d(TAG, "Value is: " + value)
            }

            override fun onCancelled(error: DatabaseError) {
                Log.w(TAG, "Failed to read value.", error.toException())
            }
            */
     */
}