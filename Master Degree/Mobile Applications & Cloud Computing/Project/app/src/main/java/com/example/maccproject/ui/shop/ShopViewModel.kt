package com.example.maccproject.ui.shop

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel

class ShopViewModel : ViewModel() {

    private val _text = MutableLiveData<String>().apply {
        value = "Loading the shop..."
    }
    private val _text2 = MutableLiveData<String>().apply {
        value = "Shopping cart: "
    }
    private val _text3 = MutableLiveData<String>().apply {
        value = ""
    }

    val loadText: LiveData<String> = _text
    val voidText: LiveData<String> = _text3
}