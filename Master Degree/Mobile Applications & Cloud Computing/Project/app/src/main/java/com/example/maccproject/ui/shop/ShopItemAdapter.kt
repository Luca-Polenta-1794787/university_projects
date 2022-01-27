package com.example.maccproject.ui.shop

import android.content.Context
import android.graphics.BitmapFactory
import android.util.Base64
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.BaseAdapter
import android.widget.ImageButton
import android.widget.ImageView
import android.widget.TextView
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentShopBinding

class ShopItemAdapter(
    private val context: Context,
    private val dataSource: ArrayList<ShopItem>,
    private val sc: ShoppingCart,
    private val binding: FragmentShopBinding
) : BaseAdapter() {

    private val inflater: LayoutInflater
            = context.getSystemService(Context.LAYOUT_INFLATER_SERVICE) as LayoutInflater

    //1
    override fun getCount(): Int {
        return dataSource.size
    }

    //2
    override fun getItem(position: Int): Any {
        return dataSource[position]
    }

    //3
    override fun getItemId(position: Int): Long {
        return position.toLong()
    }


    //4
    override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
        // Get view for row item
        val rowView = inflater.inflate(R.layout.shop_item, parent, false)
        //add here every value
        val nameTextView = rowView.findViewById(R.id.product_name) as TextView
        val priceTextView = rowView.findViewById(R.id.product_price) as TextView
        val imageImageView = rowView.findViewById(R.id.product_image) as ImageView
        val quantityView = rowView.findViewById(R.id.product_quantity) as TextView

        val shopItem = getItem(position) as ShopItem

        nameTextView.text = shopItem.itemName
        priceTextView.text = String.format("%.2f", shopItem.itemCost) + "€"
        quantityView.text = "Available: " + shopItem.itemQuantity.toString()

        val imageBytes = Base64.decode(shopItem.itemPic, Base64.DEFAULT)
        val decodedImage = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.size)
        imageImageView.setImageBitmap(decodedImage)

        //add the listeners
        val addCart = rowView.findViewById(R.id.addToCart) as ImageButton
        addCart.setOnClickListener {
            val item = CartItem(shopItem)
            sc.addItem(item)
            val cart_size : TextView = binding.cartSize
            cart_size.text = sc.getShoppingCartSize().toString()
            binding.invalidateAll()
            //val cart_size : TextView = binding.cartSize
        }

        return rowView
    }

}