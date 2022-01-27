package com.example.maccproject

import android.content.Context
import android.os.Bundle
import android.view.View
import com.google.android.material.bottomnavigation.BottomNavigationView
import androidx.appcompat.app.AppCompatActivity
import androidx.navigation.findNavController
import androidx.navigation.ui.AppBarConfiguration
import androidx.navigation.ui.setupActionBarWithNavController
import androidx.navigation.ui.setupWithNavController
import com.example.maccproject.databinding.ActivityMainBinding
import android.content.Intent
import com.example.maccproject.ui.game.GameActivity
import io.paperdb.Paper


class MainActivity : AppCompatActivity() {

    companion object {
        private lateinit var context: Context

        fun setContext(con: Context) {
            context=con
        }

        fun getContext() :Context{
            return context
        }
    }

    private lateinit var binding: ActivityMainBinding

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        binding = ActivityMainBinding.inflate(layoutInflater)
        supportActionBar?.hide()
        setContentView(binding.root)

        val navView: BottomNavigationView = binding.navView

        val navController = findNavController(R.id.nav_host_fragment_activity_main)
        // Passing each menu ID as a set of Ids because each
        // menu should be considered as top level destinations.
        val appBarConfiguration = AppBarConfiguration(
            setOf(
                R.id.navigation_home, R.id.navigation_quiz, R.id.navigation_shop, R.id.navigation_game
            )
        )
        setupActionBarWithNavController(navController, appBarConfiguration)
        navView.setupWithNavController(navController)
        //hide top bar
        navView.visibility = View.GONE
        setContext(this)

        //init paper
        Paper.init(context)
    }


    fun startGame(view: View?) {
        val intent = Intent(this, GameActivity::class.java)
        startActivity(intent)
    }

}
