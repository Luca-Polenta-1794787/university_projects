package com.example.maccproject.ui.game.engines

import android.annotation.SuppressLint
import android.content.Context
import android.util.Log
import android.view.SurfaceHolder
import android.view.SurfaceView
import com.example.maccproject.ui.game.models.Blocco
import com.example.maccproject.ui.game.models.Player
import com.example.maccproject.R
import android.graphics.*
import android.graphics.Shader
import android.graphics.BitmapShader
import android.graphics.BitmapFactory
import android.graphics.Bitmap


// Permette di gestire la visualizzazione degli elementi del gioco SensorMaze
class MazeView(pContext: Context?, player: Player) : SurfaceView(pContext), SurfaceHolder.Callback {
    var mPlayer: Player? = player
    var mSurfaceHolder: SurfaceHolder = holder
    private var mThread: DrawingThread
    private var mBackColor = Color.BLACK //Color.CYAN
    private var mBlocks: List<Blocco>? = null
    private var mPaint: Paint? = null
    private var bucoPaint: Paint? = null
    private var playerPaint: Paint? = null
    private var playerFastPaint: Paint? = null
    private var playerNightPaint: Paint? = null
    private var arrivePaint: Paint? = null
    var sfondoGiocoStelle: Bitmap
    private var sourceSfondo: Rect
    private var bitmapSfondoRect: Rect
    var sfondoGalassia: Bitmap
    var figuraPlayer: Bitmap
    var figuraPlayerFast: Bitmap
    var figuraPlayerNight: Bitmap
    var figuraArrivo: Bitmap
    private val fillBmpShaderBackgroundStars : BitmapShader
    private val fillBmpShaderGalassia: BitmapShader
    private var fillBmpShaderSpaceship: BitmapShader
    private var fillBmpShaderSpaceshipFast: BitmapShader
    private var fillBmpShaderSpaceshipNight: BitmapShader
    private val fillBmpShaderHospital: BitmapShader

    // Retourne la liste des Blocs composant la 'carte' du labyrinthe
    fun getBlocks(): List<Blocco>? {
        return mBlocks
    }

    // Cambia la lista dei blocchi che compongono la 'mappa' del labirinto
    fun setBlocks(pBlocks: List<Blocco?>?) {
        this.mBlocks = pBlocks as List<Blocco>?
    }

    // Restituisce il giocatore per la visualizzazione
    fun getPlayer(): Player? {
        return mPlayer
    }

    // Cambia il giocatore usato per la visualizzazione
    fun setPlayer(pPlayer: Player) {
        this.mPlayer = pPlayer
    }

    // Il tabellone di gioco viene disegnato quando il display viene aggiornato
    override fun onDraw(pCanvas: Canvas) {
        super.onDraw(pCanvas)

        // Disegnare prima lo sfondo dello schermo
        pCanvas.drawBitmap(sfondoGiocoStelle, sourceSfondo, bitmapSfondoRect, null)

        if (mBlocks != null) {
            // Disegna tutti i blocchi del labirinto
            for (b in mBlocks!!) {
                when (b.getType()) {
                    Blocco.Type.INIZIO -> {
                        mPaint!!.color = mBackColor //Color.WHITE
                        pCanvas.drawRect(b.getRectangle()!!, mPaint!!)
                    }
                    Blocco.Type.ARRIVO -> {
                        pCanvas.drawBitmap(figuraArrivo, b.getRectangle()!!.left,
                            b.getRectangle()!!.top, arrivePaint)
                    }
                    Blocco.Type.BUCO -> {
                        pCanvas.drawRect(b.getRectangle()!!, bucoPaint!!)
                    }
                }
            }
        }

        // Disegno il giocatore
        if (mPlayer != null) {
            when {
                mPlayer!!.getPlayerFast() -> {
                    pCanvas.drawBitmap(figuraPlayerFast, mPlayer!!.getX()-mPlayer!!.getRaggio(),
                        mPlayer!!.getY()-mPlayer!!.getRaggio(), playerFastPaint)
                }
                mPlayer!!.getPlayerNight() -> {
                    pCanvas.drawBitmap(figuraPlayerNight, mPlayer!!.getX()-mPlayer!!.getRaggio(),
                        mPlayer!!.getY()-mPlayer!!.getRaggio(), playerNightPaint)
                }
                else -> {
                    pCanvas.drawBitmap(figuraPlayer, mPlayer!!.getX()-mPlayer!!.getRaggio(),
                        mPlayer!!.getY()-mPlayer!!.getRaggio(), playerPaint)
                }
            }
        }

        invalidate()
    }

    override fun surfaceChanged(pHolder: SurfaceHolder, pFormat: Int, pWidth: Int, pHeight: Int) {}

    // Quando la superficie viene creata, il giocatore viene inizializzata
    override fun surfaceCreated(pHolder: SurfaceHolder) {
        mThread.keepDrawing = true
        mThread.start()
        // Quando il giocatore viene creata, le vengono date le coordinate dello schermo
        if (mPlayer != null) {
            mPlayer!!.setHeight(height)
            mPlayer!!.setWidth(width)
        }
    }

    // Puliamo fino alla distruzione della superficie
    override fun surfaceDestroyed(pHolder: SurfaceHolder) {
        mThread.keepDrawing = false
        var retry = true
        while (retry) {
            try {
                mThread.join()
                retry = false
            } catch (e: InterruptedException) {
                Log.e("MAZE_VIEW", e.message!!)
            }
        }
    }

    // Produttore di motori grafici
    init {
        this.mSurfaceHolder.addCallback(this)
        // Carico il background del gioco
        sfondoGiocoStelle = BitmapFactory.decodeResource(resources, R.drawable.game_background_stars)
        fillBmpShaderBackgroundStars = BitmapShader(sfondoGiocoStelle, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP)
        // Definisco le dimensioni da mettere in drawBitmap per la stampa dello sfondo
        sourceSfondo = Rect(0, 0, sfondoGiocoStelle.width, sfondoGiocoStelle.height)
        bitmapSfondoRect = Rect(0, 0, sfondoGiocoStelle.width, sfondoGiocoStelle.height)
        // Definisco la grafica generale del mondo
        mPaint = Paint()
        mPaint!!.style = Paint.Style.FILL
        //Definisco lo sfondo
        sfondoGalassia = BitmapFactory.decodeResource(resources, R.drawable.big_galaxy_hd)
        fillBmpShaderGalassia = BitmapShader(sfondoGalassia, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP)
        // Definisco la grafica dei muri
        bucoPaint = Paint()
        bucoPaint!!.color = -0x1
        bucoPaint!!.style = Paint.Style.FILL
        bucoPaint!!.shader = fillBmpShaderGalassia
        // Definisco il player
        figuraPlayer = BitmapFactory.decodeResource(resources, R.drawable.game_player)
        fillBmpShaderSpaceship = BitmapShader(figuraPlayer, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP)
        // Definisco la grafica del player
        playerPaint = Paint()
        playerPaint!!.color = -0x1
        playerPaint!!.style = Paint.Style.FILL
        playerPaint!!.shader = fillBmpShaderSpaceship
        // Definisco il player fast
        figuraPlayerFast = BitmapFactory.decodeResource(resources, R.drawable.game_player_fast)
        fillBmpShaderSpaceshipFast = BitmapShader(figuraPlayerFast, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP)
        // Definisco la grafica del player fast
        playerFastPaint = Paint()
        playerFastPaint!!.color = -0x1
        playerFastPaint!!.style = Paint.Style.FILL
        playerFastPaint!!.shader = fillBmpShaderSpaceshipFast
        // Definisco il player fast
        figuraPlayerNight = BitmapFactory.decodeResource(resources, R.drawable.game_player_night)
        fillBmpShaderSpaceshipNight = BitmapShader(figuraPlayerNight, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP)
        // Definisco la grafica del player fast
        playerNightPaint = Paint()
        playerNightPaint!!.color = -0x1
        playerNightPaint!!.style = Paint.Style.FILL
        playerNightPaint!!.shader = fillBmpShaderSpaceshipNight
        // Definisco l'arrivo
        figuraArrivo = BitmapFactory.decodeResource(resources, R.drawable.game_hospital)
        fillBmpShaderHospital = BitmapShader(figuraArrivo, Shader.TileMode.CLAMP, Shader.TileMode.CLAMP)
        // Definisco la grafica dell'arrivo
        arrivePaint = Paint()
        arrivePaint!!.color = -0x1
        arrivePaint!!.style = Paint.Style.FILL
        arrivePaint!!.shader = fillBmpShaderHospital
        // Definisco il thread che disegna il mondo
        mThread = DrawingThread()
    }

    // Classe Thread usata per il disegno
    private inner class DrawingThread() : Thread() {
        var keepDrawing = true

        @SuppressLint("WrongCall")
        override fun run() {
            var canvas: Canvas?
            while (keepDrawing) {
                canvas = null
                try {
                    canvas = mSurfaceHolder.lockCanvas(null)
                    synchronized(mSurfaceHolder) { onDraw(canvas) }
                } finally {
                    if (canvas != null) mSurfaceHolder.unlockCanvasAndPost(canvas)
                }

                try {
                    sleep(2) // C'ERA "20" ED ERA MOTIVATO "PER DISEGNARE A 50FPS", MA ORA IL GIOCATORE È PIÙ LENTO
                } catch (e: InterruptedException) {
                    Log.e("GAME_VIEW", e.message!!)
                }
            }
        }

    }

}