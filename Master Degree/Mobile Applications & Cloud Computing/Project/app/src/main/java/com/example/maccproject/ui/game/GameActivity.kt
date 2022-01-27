package com.example.maccproject.ui.game


import android.content.Context
import android.graphics.Point
import android.os.Build
import androidx.appcompat.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import android.view.WindowInsets
import android.view.WindowManager
import androidx.annotation.RequiresApi
import com.example.maccproject.ui.game.engines.MazeEngine
import com.example.maccproject.ui.game.engines.MazeView
import com.example.maccproject.ui.game.models.Blocco
import com.example.maccproject.ui.game.models.Player
import android.widget.Toast
import com.google.android.gms.tasks.Task
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.database.DataSnapshot
import com.google.firebase.database.FirebaseDatabase


class GameActivity : AppCompatActivity() {
    // Il motore fisico del gioco
    private var mEngine: MazeEngine? = null

    private val tagGameActivity :String = "GameActivity"
    private var numVictories = 0

    val currentUser = FirebaseAuth.getInstance().currentUser
    val database = FirebaseDatabase.getInstance()
    val snapshot: Task<DataSnapshot> = database.reference.child("Users").child(currentUser?.uid.toString()).get().addOnSuccessListener {
        Log.i("firebase", "Got value ${it.value}")
        val string:String = it.value.toString()
        if(string.contains("game")){
            val num_victories_str = string.filter { it.isDigit() }
            numVictories = num_victories_str.toInt()
            Log.d("firebase", "Got number of victories equal to $numVictories")
        }
        else{
            Log.d("firebase", "No victories yet!")
        }
    }.addOnFailureListener{
        Log.e("firebase", "Error getting data", it)
    }

    // Quando l'attività viene creata, vengono inizializzati i seguenti elementi
    // i motori di gioco e tutte le dipendenze.
    @RequiresApi(Build.VERSION_CODES.R)
    override fun onCreate(savedInstanceState: Bundle?) { // savedInstanceState: Stato salvato dell'applicazione
        super.onCreate(savedInstanceState)

        // Creo il giocatore
        val b = Player(32)

        // DA QUI CREO LA GRAFICA DEL GIOCO E AVVIO I CHECK DELLE COLLISIONI
        val mView = MazeView(this, b)
        mView.setPlayer(b)
        mView.setBlocks(null)

        // DA QUI CREO LA GESTIONE DELLA PARTE FISICA DEL GIOCO
        mEngine = MazeEngine(this)
        mEngine!!.setPlayer(b)
        // MA DALL'API 30 È STATA DEPRECATA, QUINDI ORA C'È DA DISTINGUERE PER API>=30 E <30
        val wm = this.getSystemService(Context.WINDOW_SERVICE) as WindowManager
        val width: Int
        val height: Int
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            val windowMetrics = wm.currentWindowMetrics
            val windowInsets: WindowInsets = windowMetrics.windowInsets

            val insets = windowInsets.getInsetsIgnoringVisibility(
                WindowInsets.Type.navigationBars() or WindowInsets.Type.displayCutout())
            val insetsWidth = insets.right + insets.left
            val insetsHeight = insets.top + insets.bottom

            width = windowMetrics.bounds.width() - insetsWidth
            height = windowMetrics.bounds.height() - insetsHeight
        } else {
            // PRIMA SI USAVA DIRETTAMENTE:
                // windowManager.defaultDisplay.getSize(screenSizes)
            val size = Point()
            val display = wm.defaultDisplay // deprecated in API 30
            display?.getSize(size) // deprecated in API 30
            width = size.x
            height = size.y
        }
        val mList: List<Blocco>? = mEngine!!.buildingTheMaze(width, height)

        // Setto la mappa per come è fatta
        mView.setBlocks(mList)

        assert(supportActionBar != null)//null check
        supportActionBar!!.setDisplayHomeAsUpEnabled(true) //show back button

        mEngine!!.setCheckAllCreated(true)

        setContentView(mView)
    }

    override fun onSupportNavigateUp(): Boolean {
        finish()
        return true
    }

    // Per riprendere il gioco.
    override fun onResume() {
        super.onResume()
        mEngine!!.resume()
    }

    // Per mettere in pausa il gioco.
    override fun onPause() {
        super.onPause()
        super.onStop()
        mEngine!!.stop()
    }

    // Per visualizzare il dialogo della vittoria, permette di riavviare l'heu se si clicca su "Start over".
    fun showVictoryDialog() {
        mEngine!!.stop()
        //Save number of victories
        val mAuth = FirebaseAuth.getInstance()
        val currentUser = mAuth.currentUser
        val database = FirebaseDatabase.getInstance()
        Log.d(tagGameActivity, "User: " + currentUser?.uid.toString())
        numVictories++
        database.reference.child("Users").child(currentUser?.uid.toString()).child("game").setValue(numVictories.toString())
        Toast.makeText(this,"Result saved correctly" , Toast.LENGTH_SHORT).show()
        GameDialogFragment.newInstance(GameDialogFragment.DialogType.VICTORY_DIALOG) { _, _ -> // L'utente può ricominciare se vuole
            mEngine!!.reset()
            mEngine!!.resume()
        }.show(this.supportFragmentManager, "GameDialog")
    }

    // Per visualizzare il dialogo della sconfitta, permette di riavviare l'heu se si clicca su "Restart".
    fun showDefeatDialog() {
        mEngine!!.stop()
        GameDialogFragment.newInstance(GameDialogFragment.DialogType.DEFEAT_DIALOG) { _, _ -> // L'utente può ricominciare se vuole
            mEngine!!.reset()
            mEngine!!.resume()
        }.show(this.supportFragmentManager, "GameDialog")
    }
}