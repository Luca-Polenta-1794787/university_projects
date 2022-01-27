package com.example.myapplication.modelsForServerCommunications

data class GamePosition(val id: String, val lat: String, val lon: String, val idOpponent:String?) {
    // Null default values create a no-argument default constructor, which is needed
    // for deserialization from a DataSnapshot.

    // Default constructor required for calls to dataSnapshot.getValue<GamePosition>()
    constructor() : this("", "", "", null)

}