package com.example.maccproject.ui.shop

import io.paperdb.Paper

class ShoppingCart {

    //consider changing .product.itemName with some id!!!
        fun addItem(cartItem: CartItem) {
            val cart = getCart()

        //Log.i("info", cartItem.product.itemPic.toString())

            val targetItem = cart.singleOrNull { it.product.itemName == cartItem.product.itemName }
            if (targetItem == null) {
                cartItem.quantity++
                cart.add(cartItem)
            } else {
                targetItem.quantity++
            }
            saveCart(cart)
        }

        fun removeItem(cartItem: CartItem) {
            val cart = getCart()

            val targetItem = cart.singleOrNull { it.product.itemName == cartItem.product.itemName }
            if (targetItem != null) {
                if (targetItem.quantity > 0) {
                    targetItem.quantity--
                } else {
                    cart.remove(targetItem)
                }
            }

            saveCart(cart)
        }

        fun saveCart(cart: MutableList<CartItem>) {
            Paper.book().write("cart", cart)
        }

        fun getCart(): MutableList<CartItem> {
            return Paper.book().read("cart", mutableListOf())
        }

        fun getShoppingCartSize(): Int {
            var cartSize = 0
            getCart().forEach {
                cartSize += it.quantity
            }

            return cartSize
        }

        fun countItem(item: ShopItem): Int{
            val cart = getCart()
            var counter = 0
            for(it in cart){
                if(it.product.itemName == item.itemName) {
                    counter += it.quantity
                }
            }

            return counter
        }

        fun totalCost(): Float{
            val cart = getCart()
            var counter = 0f
            for(it in cart){
                counter += it.quantity * it.product.itemCost
            }


            return counter
        }

        //use this function if you modified something and need to empty the cart
        fun emptyCart(){
            while(getCart().size != 0) {
                for (i in getCart()) {
                    removeItem(i)
                }
            }
        }

}