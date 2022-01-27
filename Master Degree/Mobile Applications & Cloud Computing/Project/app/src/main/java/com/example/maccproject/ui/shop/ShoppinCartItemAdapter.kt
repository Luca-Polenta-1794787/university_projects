package com.example.maccproject.ui.shop

import android.content.Context
import android.graphics.BitmapFactory
import android.util.Base64
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.*
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentShoppingCartBinding

class ShoppinCartItemAdapter(
    private val context: Context,
    private val dataSource: ArrayList<ShopItem>,
    private val binding: FragmentShoppingCartBinding
) : BaseAdapter() {

    private val inflater: LayoutInflater =
        context.getSystemService(Context.LAYOUT_INFLATER_SERVICE) as LayoutInflater

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
        val rowView = inflater.inflate(R.layout.sopping_cart_item, parent, false)
        //add here every value
        val nameTextView = rowView.findViewById(R.id.product_name_cart) as TextView
        val priceTextView = rowView.findViewById(R.id.product_price_cart) as TextView
        val imageImageView = rowView.findViewById(R.id.product_image_cart) as ImageView
        val quantityTextView = rowView.findViewById(R.id.quantity) as TextView
        val totalTextView = rowView.findViewById(R.id.total) as TextView

        val sc = ShoppingCart()

        val shopItem = getItem(position) as ShopItem

        nameTextView.text = shopItem.itemName
        priceTextView.text = String.format("%.2f", shopItem.itemCost) + "€"
        quantityTextView.text = sc.countItem(shopItem).toString()
        totalTextView.text = String.format("%.2f", shopItem.itemCost * sc.countItem(shopItem)) + "€"
        //manage the image and the other values

        val imageBytes = Base64.decode(shopItem.itemPic, Base64.DEFAULT)
        val decodedImage = BitmapFactory.decodeByteArray(imageBytes, 0, imageBytes.size)
        imageImageView.setImageBitmap(decodedImage)

        val plus = rowView.findViewById(R.id.button_plus) as Button
        plus.setOnClickListener{
            val item = CartItem(shopItem)
            sc.addItem(item)
            quantityTextView.text = sc.countItem(shopItem).toString()
            totalTextView.text = String.format("%.2f", shopItem.itemCost * sc.countItem(shopItem)) + "€"
            binding.total.text = "Total: " + sc.totalCost().toString() + " €"
        }

        val minus = rowView.findViewById(R.id.button_minus) as Button
        minus.setOnClickListener{
            val item = CartItem(shopItem)
            sc.removeItem(item)
            quantityTextView.text = sc.countItem(shopItem).toString()
            totalTextView.text = String.format("%.2f", shopItem.itemCost * sc.countItem(shopItem)) + "€"
            binding.total.text = "Total: " + sc.totalCost().toString() + " €"
        }

        return rowView
    }
}

