package com.example.maccproject.ui.quiz

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class QuizViewModel : ViewModel() {

    private val _quizTitle = MutableLiveData<String>().apply {
        value = "Welcome to the Star Wars Quiz!"
    }
    val quizTitle: LiveData<String> = _quizTitle

    private val _quizGeneralInfo = MutableLiveData<String>().apply {
        value = "Find out which of the two powerful sides of the force you belong to! In this quiz " +
                "you will be shown various images. Click on the ones you like most, confirm your " +
                "choice and find out whether you are a Jedi or a Sith."
    }
    val quizGeneralInfo: LiveData<String> = _quizGeneralInfo

}