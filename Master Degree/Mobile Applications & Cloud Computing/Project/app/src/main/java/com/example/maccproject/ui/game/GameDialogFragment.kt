package com.example.maccproject.ui.game

import android.app.AlertDialog
import android.app.Dialog
import androidx.fragment.app.DialogFragment
import android.content.DialogInterface
import android.os.Bundle
import java.lang.NullPointerException

// Questa classe è responsabile della visualizzazione dei dialoghi di vittoria o sconfitta del gioco.
class GameDialogFragment : DialogFragment() {
    // Il callback da chiamare quando viene premuto il pulsante "Restart".
    var mRestartCallback: DialogInterface.OnClickListener? = null

    // I diversi tipi di dialogo disponibili.
    enum class DialogType {
        VICTORY_DIALOG, DEFEAT_DIALOG
    }

    // Il tipo di dialogo selezionato.
    private var mDialogType: DialogType? = null

    // Quando si crea il dialogo, i dati vengono visualizzati secondo il tipo.
    override fun onCreateDialog(savedInstanceState: Bundle?): Dialog {
        var title = ""
        var message = ""
        if (mDialogType == null) throw NullPointerException("Dialog type was not set!")
        when (mDialogType) {
            DialogType.VICTORY_DIALOG -> {
                title = "Hai vinto!"
                message = "Ben fatto, hai vinto!"
            }
            DialogType.DEFEAT_DIALOG -> {
                title = "Sconfitta"
                message = "Peccato, hai perso..."
            }
        }
        return AlertDialog.Builder(activity)
            .setTitle(title)
            .setMessage(message)
            .setNeutralButton("Ricomincia", mRestartCallback)
            .create()
    }

    // Cambia il tipo di dialogo da visualizzare.
    private fun setType(type: DialogType) {
        mDialogType = type
    }

    companion object {
        // Crea una nuova istanza di Fragemnt.
        fun newInstance(
            type: DialogType, // Tipo di dialogo da generare.
            callback: DialogInterface.OnClickListener? // Callback per il pulsante "Restart
        ): GameDialogFragment {
            val fragment = GameDialogFragment()
            fragment.setType(type)
            fragment.mRestartCallback = callback
            return fragment
        }
    }
}