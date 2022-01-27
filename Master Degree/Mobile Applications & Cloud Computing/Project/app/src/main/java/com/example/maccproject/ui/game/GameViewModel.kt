package com.example.maccproject.ui.game


import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class GameViewModel : ViewModel() {

    private val _title = MutableLiveData<String>().apply {
        value = "Welcome to the new STAR WARS game!"
    }
    val title: LiveData<String> = _title

    private val _explanation = MutableLiveData<String>().apply {
        value = "Are you ready to put yourself to the challenge? In this game tilt your phone to " +
                "command your space shuttle and bring urgent supplies to the space hospital! The " +
                "ship stops when the phone is parallel to the floor, and the game works with the " +
                "screen up or down. If you tilt your phone a lot, the ship goes into turbo mode " +
                "with a whole new look, and when there's little light in the room it turns on its " +
                "headlights.\nFinally, every time you win, it will be saved to your personal " +
                "account to keep track of your progress."
    }
    val explanation: LiveData<String> = _explanation
}