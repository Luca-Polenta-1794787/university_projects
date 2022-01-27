package com.example.maccproject.ui.game.models

import android.graphics.RectF
import kotlin.math.sign

class Player(raggio: Int) {
    // Velocità massima consentita del giocatore
    private val MAX_SPEED = 2.5f // PRIMA 20.0f

    // Permette al giocatore di accelerare più lentamente
    private val compensatore_accellerazione = 8.0f

    // Raggio del giocatore
    private var mRaggio = raggio

    // Il rettangolo di collisione del giocatore
    private var mRectangle: RectF? = null

    private var playerFast = false
    private var playerNight = false

    // Coordinata della X
    private var mX = 0f

    // Coordinata della Y
    private var mY = 0f

    // Coordinata iniziale della X
    private var initialmX = 0f

    // Coordinata inizialedella Y
    private var initialmY = 0f

    // Velocità dell'asse X
    private var mSpeedX = 0f

    // Velocità dell'asse Y
    private var mSpeedY = 0f

    // Dimensioni dello schermo in altezza.
    private var mHeight = -1

    // Dimensione dello schermo in larghezza.
    private var mWidth = -1

    // Restituisce il raggio del giocatore
    fun getRaggio(): Int {
        return mRaggio
    }

    // Dal rettangolo iniziale si determina la posizione del giocatore
    fun setInitialRectangle(pInitialRectangle: RectF) {
        mX = pInitialRectangle.left + mRaggio/2
        mY = pInitialRectangle.top + mRaggio/2
        initialmX = mX
        initialmY = mY
    }

    // Restituisce la posizione X del giocatore
    fun getX(): Float {
        return mX
    }

    // Cambia la posizione X del giocatore
    private fun setPosX(pPosX: Float) {
        mX = pPosX
    }

    // Restituisce la posizione X del giocatore
    fun getY(): Float {
        return mY
    }

    // Restituisce la posizione Y del giocatore
    private fun setPosY(pPosY: Float) {
        mY = pPosY
    }

    // Usato quando rimbalza sulle pareti orizzontali
    fun changeXSpeed() {
        mSpeedX = -mSpeedX
    }

    // Usato quando rimbalza sulle pareti verticali
    fun changeYSpeed() {
        mSpeedY = -mSpeedY
    }

    // Cambia l'altezza dello schermo associato al giocatore
    fun setHeight(pHeight: Int) {
        mHeight = pHeight
    }

    // Cambia la larghezza dello schermo associato al giocatore
    fun setWidth(pWidth: Int) {
        mWidth = pWidth
    }


    var previousSpeedX = 0.0f
    var previousSpeedY = 0.0f
    var fermaX = false
    var fermaY = false

    // Permette di aggiornare le coordinate del giocatore
    fun putXAndY(pX: Float, pY: Float): RectF? {

        if(pX>-0.34f && pX<0.34f && !fermaX){
            if(previousSpeedX*pX<0){
                // Sono dal lato opposto dello 0
                mSpeedX += pX / compensatore_accellerazione
                if(mSpeedX*previousSpeedX>0){ // Quando diventa dello stesso segno, mi fermo
                    mSpeedX = 0f
                    fermaX = true
                }
            }else if(previousSpeedX*pX>0){
                // Sono dallo stesso segno
                mSpeedX -= pX / compensatore_accellerazione
                if(mSpeedX*previousSpeedX<0){ // Se cambia segno, mi fermo
                    mSpeedX = 0f
                    fermaX = true
                }
            }else{ // Qui pX è uguale a 0!!
                mSpeedX -= sign(previousSpeedX)*0.34f / compensatore_accellerazione
                if(mSpeedX*previousSpeedX<0){ // Quando cambia segno, mi fermo
                    mSpeedX = 0f
                    fermaX = true
                }
            }
        }else{
            fermaX = false
            mSpeedX += pX / compensatore_accellerazione
            if (mSpeedX > MAX_SPEED) mSpeedX = MAX_SPEED
            if (mSpeedX < -MAX_SPEED) mSpeedX = -MAX_SPEED
        }
        previousSpeedX=mSpeedX


        if(pY>-0.34f && pY<0.34f && !fermaY){
            if(previousSpeedY*pY<0){
                // Sono dal lato opposto dello 0
                mSpeedY += pY / compensatore_accellerazione
                if(mSpeedY*previousSpeedY>0){ // Quando diventa dello stesso segno, mi fermo
                    mSpeedY = 0f
                    fermaY = true
                }
            }else if(previousSpeedY*pY>0){
                // Sono dallo stesso segno
                mSpeedY -= pY / compensatore_accellerazione
                if(mSpeedY*previousSpeedY<0){ // Se cambia segno, mi fermo
                    mSpeedY = 0f
                    fermaY = true
                }
            }else{ // Qui pY è uguale a 0!!
                mSpeedY -= sign(previousSpeedY)*0.34f / compensatore_accellerazione
                if(mSpeedY*previousSpeedY<0){ // Quando cambia segno, mi fermo
                    mSpeedY = 0f
                    fermaY = true
                }
            }
        }else{
            fermaY = false
            mSpeedY += pY / compensatore_accellerazione
            if (mSpeedY > MAX_SPEED) mSpeedY = MAX_SPEED
            if (mSpeedY < -MAX_SPEED) mSpeedY = -MAX_SPEED
        }
        previousSpeedY=mSpeedY


        setPosX(mX + mSpeedY)
        setPosY(mY + mSpeedX)

        // Aggiorna le coordinate del rettangolo di collisione
        mRectangle!!.set(mX - mRaggio, mY - mRaggio, mX + mRaggio, mY + mRaggio)
        return mRectangle // nuovo rettangolo di collisione
    }

    // Permette di riportare il giocatore nella posizione di partenza
    fun reset() {
        mSpeedX = 0f
        mSpeedY = 0f
        mX = initialmX
        mY = initialmY
    }

    fun setPlayerFast(b: Boolean) {
        playerFast=b
    }

    fun getPlayerFast():Boolean{
        return playerFast
    }

    fun setPlayerNight(b: Boolean) {
        playerNight=b
    }

    fun getPlayerNight():Boolean{
        return playerNight
    }

    // Costruttore dell'oggetto Player
    init {
        mRectangle = RectF()
    }
}