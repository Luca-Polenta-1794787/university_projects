package com.example.maccproject.ui.shop

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.ListView
import android.widget.TextView
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.findNavController
import com.example.maccproject.MainActivity
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentShopBinding
import kotlinx.coroutines.*
import org.json.JSONArray
import org.json.JSONObject
import java.io.InputStreamReader
import java.net.HttpURLConnection
import java.net.URL

class ShopFragment : Fragment() {

    private lateinit var shopViewModel: ShopViewModel
    private var _binding: FragmentShopBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        shopViewModel =
            ViewModelProvider(this)[ShopViewModel::class.java]

        _binding = FragmentShopBinding.inflate(inflater, container, false)
        val root: View = binding.root

        binding.shop = this

        val textView: TextView = binding.textShop
        shopViewModel.loadText.observe(viewLifecycleOwner, {
            textView.text = it
        })

        val sc = ShoppingCart()
        //uncomment if you want to empty the shopping cart
        //it has side-effects on the db, be careful
        //sc.emptyCart()

        val cart_size : TextView = binding.cartSize
        cart_size.text = sc.getShoppingCartSize().toString()


        //retrieve the list
        binding.basketButton.setOnClickListener { v: View ->
            v.findNavController().navigate(R.id.action_navigation_shop_to_shop_cart)
        }

        GlobalScope.launch {
            val c1 = async { doGet() }
            withContext(Dispatchers.Main) {
                //add here the code to update the views (this is going to be executed as main)
                val itemList = c1.await()
                //Log.i("info","exit, size = " + itemList.size)
                val listView: ListView = binding.itemListView
                val adapter = ShopItemAdapter(MainActivity.getContext(), itemList, sc, binding)
                listView.adapter = adapter

                shopViewModel.voidText.observe(viewLifecycleOwner, {
                    textView.text = it
                })
            }
        }

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    private fun doGet() : ArrayList<ShopItem> {
        val mURL = URL("https://heinzeen.pythonanywhere.com")
        val itemList = arrayListOf<ShopItem>()

        with(mURL.openConnection() as HttpURLConnection) {
            // optional default is GET
            requestMethod = "GET"
            val jsonarray = JSONArray(InputStreamReader(inputStream).readText())

            //Log.i("info",jsonarray.toString())

            for (i in 0 until jsonarray.length()) {
                val jsonobject: JSONObject = jsonarray.getJSONObject(i)
                val name = jsonobject.getString("name").replace("_", " ")
                val cost = jsonobject.getString("cost")
                val quantity = jsonobject.getString("quantity")
                val img = jsonobject.getString("img")
                itemList.add(ShopItem(name, cost.toFloat(), img, quantity.toInt()))
            }

            //for debugging purposes
            //println("URL : $url")
            //println("Response Code : $responseCode")

        }

        return itemList
    }

}