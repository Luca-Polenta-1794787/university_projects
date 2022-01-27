package com.example.maccproject.ui.game.engines

import android.app.Service
import android.graphics.RectF
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.util.Log
import com.example.maccproject.ui.game.GameActivity
import com.example.maccproject.ui.game.models.Blocco
import com.example.maccproject.ui.game.models.Player
import java.util.ArrayList

/**
 * Questa classe gestisce la parte fisica del gioco SensorMaze.
 * Si occupa anche dei diversi sensori utilizzati e aggiorna i valori che dipendono da essi.
 * Il motore si occupa anche di creare i blocchi e il giocatore tramite una funzione statica.
 */
class MazeEngine(pView: GameActivity) { // Riferimento all'attività chiamante
    private var mPlayer: Player? = null

    private var mBlocks: MutableList<Blocco>? = null

    private var checkAllCreated = false

    private var mActivity: GameActivity? = pView

    private var mManager: SensorManager? = null
    private var mAccelerometre: Sensor? = null
    private var mMagneticSensor: Sensor? = null
    private var mLightSensor: Sensor? = null

    fun setCheckAllCreated(b: Boolean){
        checkAllCreated = b
    }

    // Listener collegato all'accelerometro, gestisce il movimento del giocatore
    private val mAccelerometerSensorEventListener: SensorEventListener =
        object : SensorEventListener {

            override fun onSensorChanged(pEvent: SensorEvent) {
                val x = pEvent.values[0]
                val y = pEvent.values[1]

                if (mPlayer != null && checkAllCreated) {
                    // Aggiorniamo le coordinate del giocatore
                    val hitBox: RectF? = mPlayer!!.putXAndY(y, -x)

                    // Per tutti i blocchi del labirinto
                    for (block in mBlocks!!) {
                        // Si crea un nuovo rettangolo per non modificare quello del blocco
                        val inter = RectF(block.getRectangle())
                        if (inter.intersect(hitBox!!)) {
                            // Vengono intraprese diverse azioni a seconda del tipo di blocco
                            when (block.getType()) {
                                Blocco.Type.BUCO -> mActivity!!.showDefeatDialog()
                                Blocco.Type.INIZIO -> {}
                                Blocco.Type.ARRIVO -> mActivity!!.showVictoryDialog()
                            }
                            break
                        }
                    }
                }
            }

            override fun onAccuracyChanged(pSensor: Sensor, pAccuracy: Int) {}
        }


    // Listener collegato al sensore magnetico, gestisce il cambiamento di colore del giocatore
    private val mMagneticSensorEventListener: SensorEventListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            val x = event.values[0]
            val y = event.values[1]
            val z = event.values[2]
            if(mPlayer!=null && checkAllCreated){
                // Controllo quanto è inclinato il telefono e in base a quello decido se cambiare player in fastPlayer
                if(
                    ((x in -1.53f..1.53f) && (y in 38.97f..49.00f) && (z in -19.09f..29.31f)) || // Inclinazione X-Rot <-120 o >-60 & Y-Rot <-15 o >15 (telefono faccia in su)
                    ((x in -1.53f..1.53f) && (y in -49.00f..-38.97f) && (z in -29.31f..19.09f)) // Inclinazione X-Rot <60 o >120 & Y-Rot <-15 o >15 (telefono faccia in giù)
                ){
                    mPlayer!!.setPlayerFast(false)
                }else{
                    mPlayer!!.setPlayerFast(true)
                }
            }
        }

        override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
    }

    private val floatThresholdLight = 300f
    /*
    0.002 lux	            Moonless clear night sky
    0.2 lux	                Design minimum for emergency lighting (AS2293).
    0.27 – 1 lux	        Full moon on a clear night
    3.4 lux	                Dark limit of civil twilight under a clear sky
    50 lux	                Family living room
    80 lux	                Hallway/toilet
    100 lux	                Very dark overcast day
    300 – 500 lux	        Sunrise or sunset on a clear day. Well-lit office area.
    1,000 lux	            Overcast day; typical TV studio lighting
    10,000 – 25,000 lux	    Full daylight (not direct sun)
    32,000 – 40,000 lux	    Direct sunlight
    */

    // Listener collegato al sensore di luminosità, gestisce il cambiamento del colore dello sfondo.
    private val mLightSensorEventListener: SensorEventListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent) {
            val floatSensorValue = event.values[0] // Lux
            if(checkAllCreated){
                if (floatSensorValue < floatThresholdLight) {
                    mPlayer!!.setPlayerNight(true)
                }else{
                    mPlayer!!.setPlayerNight(false)
                }
            }

        }

        override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
    }

    // Recupera la Player usata dal motore fisico
    fun getPlayer(): Player? {
        return mPlayer
    }

    // Cambia la Player usata dal motore fisico
    fun setPlayer(pPlayer: Player?) {
        mPlayer = pPlayer
    }

    // Ripristina la posizione del giocatore
    fun reset() {
        mPlayer!!.reset()
    }

    // Ferma i sensori
    fun stop() {
        mManager!!.unregisterListener(mAccelerometerSensorEventListener, mAccelerometre)
        mManager!!.unregisterListener(mMagneticSensorEventListener, mMagneticSensor)
        mManager!!.unregisterListener(mLightSensorEventListener, mLightSensor)
    }

    // Riavvia i sensori
    fun resume() {
        mManager!!.registerListener(
            mAccelerometerSensorEventListener,
            mAccelerometre,
            SensorManager.SENSOR_DELAY_GAME
        )
        mManager!!.registerListener(
            mMagneticSensorEventListener,
            mMagneticSensor,
            SensorManager.SENSOR_DELAY_GAME
        )
        mManager!!.registerListener(
            mLightSensorEventListener,
            mLightSensor,
            SensorManager.SENSOR_DELAY_GAME
        )
    }

    // Costruttore del labirinto
    fun buildingTheMaze(widht:Int, height:Int): // Dimensione dello schermo
            List<Blocco>? {
        // computa la dimensione dello schermo
        val tileSizeX = widht / 20.toFloat()
        val tileSizeY = height / 14.toFloat()
        mBlocks = ArrayList<Blocco>()
        // Aggiungo player
        val b = Blocco(Blocco.Type.INIZIO, 2, 2, tileSizeX, tileSizeY)
        mPlayer!!.setInitialRectangle(RectF(b.getRectangle()))
        mBlocks!!.add(b)
        // Aggiungo arrivo
        val checkArrivo = true
        mBlocks!!.add(Blocco(Blocco.Type.ARRIVO, 11, 10, tileSizeX, tileSizeY, checkArrivo))
        // Aggiungo ostacoli
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 1, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 2, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 3, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 4, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 6, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 7, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 9, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 10, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 11, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 0, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 1, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 1, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 1, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 2, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 2, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 2, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 3, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 3, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 3, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 1, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 2, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 3, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 4, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 6, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 7, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 9, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 4, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 5, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 5, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 5, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 6, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 6, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 6, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 7, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 7, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 7, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 6, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 9, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 10, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 11, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 8, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 9, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 9, 1, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 9, 2, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 9, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 9, 9, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 9, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 9, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 10, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 10, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 10, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 10, 9, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 10, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 10, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 11, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 11, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 11, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 11, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 11, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 12, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 12, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 12, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 12, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 12, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 13, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 13, 4, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 13, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 13, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 13, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 13, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 14, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 14, 4, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 14, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 14, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 14, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 14, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 3, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 4, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 6, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 7, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 15, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 16, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 16, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 16, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 17, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 17, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 17, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 18, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 18, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 18, 13, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 0, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 1, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 2, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 3, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 4, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 5, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 6, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 7, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 8, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 9, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 10, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 11, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 12, tileSizeX, tileSizeY))
        mBlocks!!.add(Blocco(Blocco.Type.BUCO, 19, 13, tileSizeX, tileSizeY))
        return mBlocks // Blocco generato, corrisponde alla 'mappa' del labirinto
    }

    // Costruttore dei motori fisici inizializzando i sensori.
    init {
        mManager = mActivity!!.baseContext.getSystemService(Service.SENSOR_SERVICE) as SensorManager
        mAccelerometre = mManager!!.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)
        mMagneticSensor = mManager!!.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
        mLightSensor = mManager!!.getDefaultSensor(Sensor.TYPE_LIGHT)
    }
}
