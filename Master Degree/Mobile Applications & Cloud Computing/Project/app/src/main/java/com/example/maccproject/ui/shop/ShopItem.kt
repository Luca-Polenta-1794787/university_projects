package com.example.maccproject.ui.shop

class ShopItem(name: String, cost: Float, img: String, quantity: Int) {

    val itemName = name
    val itemCost = cost
    //we save the image ad a base64 string, then we convert it everytime we need it.
    //since it has to be written in a db, we cannot save object references, base64 will do...
    val itemPic = img
    val itemQuantity = quantity

}