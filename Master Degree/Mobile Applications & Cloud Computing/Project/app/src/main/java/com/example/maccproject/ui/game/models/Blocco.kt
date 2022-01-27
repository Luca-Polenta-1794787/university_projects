package com.example.maccproject.ui.game.models

import android.graphics.RectF

/**
 * Classe "Blocco"
 * Questa classe modella un blocco sullo schermo, i blocchi possono essere buchi, l'inizio o la fine.
 */
class Blocco(pType: Type?, pX: Int, pY: Int, width: Float, height: Float, boolArrivo: Boolean? = false) {
    // I diversi tipi di blocchi disponibili.
    enum class Type {
        BUCO, INIZIO, ARRIVO
    }

    private var mWidth = width
    private var mHeight = height

    private var mType: Type? = pType
    private var mRectangle: RectF? = null

    // Restituisce la larghezza del blocco.
    fun getWidth(): Float {
        return mWidth
    }

    // Restituisce l'altezza del blocco.
    fun getHeight(): Float {
        return mHeight
    }

    // Restituisce il tipo di blocco.
    fun getType(): Type? {
        return mType
    }

    // Restituisce il rettangolo associato al blocco, utilizzato per rilevare le collisioni
    fun getRectangle(): RectF? {
        return mRectangle
    }

    /*
     * Costruttore dell'oggetto Blocco
     * @param pType Tipo di blocco.
     * @param pX posizione X in relazione alla griglia.
     * @param pY posizione Y in relazione alla griglia.
     * @param width Larghezza dell'elemento.
     * @param height Altezza dell'elemento.
     */
    init {
        // SE IL BLOCCO È FINALE, ALLORA LO FACCIO LEGGERMENTE PIÙ LARGO IN QUANTO IL DISEGNO È PIÙ LARGO
        when(boolArrivo){
            true -> {
                mRectangle = RectF(pX * mWidth, pY * mHeight, (pX + 2) * mWidth, (pY + 1) * mHeight)
            }
            else -> {
                mRectangle = RectF(pX * mWidth, pY * mHeight, (pX + 1) * mWidth, (pY + 1) * mHeight)

            }
        }
    }
}