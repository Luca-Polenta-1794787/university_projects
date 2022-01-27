package com.example.maccproject.ui.shop

import android.os.Bundle
import android.os.Looper
import android.view.LayoutInflater
import android.view.MenuItem
import android.view.View
import android.view.ViewGroup
import android.widget.*
import androidx.activity.OnBackPressedCallback
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import androidx.navigation.findNavController
import androidx.navigation.fragment.findNavController
import com.example.maccproject.MainActivity
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentShoppingCartBinding
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationServices
import kotlinx.coroutines.*
import java.io.BufferedReader
import java.io.InputStreamReader
import java.io.OutputStreamWriter
import java.net.HttpURLConnection
import java.net.URL
import java.net.URLEncoder


class ShoppingCartFragment : Fragment() {

    private lateinit var shopViewModel: ShopViewModel
    private var _binding: FragmentShoppingCartBinding? = null

    private lateinit var fusedLocationClient: FusedLocationProviderClient

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

        _binding = FragmentShoppingCartBinding.inflate(inflater, container, false)
        val root: View = binding.root

        binding.shop = this

        //location
        fusedLocationClient = LocationServices.getFusedLocationProviderClient(MainActivity.getContext())
        //items
        val sc = ShoppingCart()

        val lista :ArrayList<ShopItem> = ArrayList()

        for(i in sc.getCart()){
            //we don't need quantity for this dummy object, set it to zero
            if(i.quantity > 0) {
                val s = ShopItem(i.product.itemName, i.product.itemCost, i.product.itemPic, 0)
                lista.add(s)
            }
        }

        //convert stuff from shopping cart to shopitem

        val listView: ListView = binding.itemListViewCart
        val adapter = ShoppinCartItemAdapter(MainActivity.getContext(), lista, binding)
        listView.adapter = adapter

        val totalCost = binding.total
        totalCost.text = "Total: " + String.format("%.2f", sc.totalCost()) + "€"

        val proceed = binding.proceed
        proceed.setOnClickListener{
            GlobalScope.launch {
                val c1 = async { sendPostRequest() }
                val ret = c1.await()
                if (Looper.myLooper() == null)
                    Looper.prepare()
                if(ret.contains("ok")) {
                    Toast.makeText(context, "Order complete", Toast.LENGTH_SHORT).show()
                    sc.emptyCart()
                    withContext(Dispatchers.Main) {
                        it.findNavController().navigate(R.id.action_navigation_shop_cart_to_shop)
                    }
                }
                else if(ret.contains("TOO"))
                    Toast.makeText(context, "Check quantities!!!" , Toast.LENGTH_SHORT).show()
                else
                    Toast.makeText(context, "Server error." , Toast.LENGTH_SHORT).show()

            }
        }

        //To enabled the back button
        setHasOptionsMenu(true)

        // Handle button back on the top menu
        activity?.onBackPressedDispatcher?.addCallback(viewLifecycleOwner, object : OnBackPressedCallback(true) {
            override fun handleOnBackPressed() {
                // in here you can do logic when backPress is clicked
                root.findNavController().navigate(R.id.action_navigation_shop_cart_to_shop)
            }
        })

        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    override fun onOptionsItemSelected(item: MenuItem): Boolean {
        when (item.itemId) {
            android.R.id.home -> findNavController().navigate(R.id.navigation_shop)
        }
        return true
    }

    private fun sendPostRequest(): String{

        //items
        val sc = ShoppingCart()
        var res = ""

        var reqParam = URLEncoder.encode("order", "UTF-8") + "="

        for(i in sc.getCart()){
            reqParam += URLEncoder.encode("+" + i.product.itemName.replace(" ", "_")
                    + "-" + i.quantity, "UTF-8")
        }

        val mURL = URL("https://heinzeen.pythonanywhere.com")

        with(mURL.openConnection() as HttpURLConnection) {
            // optional default is GET
            requestMethod = "POST"

            val wr = OutputStreamWriter(outputStream)
            wr.write(reqParam)
            wr.flush()
            //println("Response Code : $responseCode")

            BufferedReader(InputStreamReader(inputStream)).use {
                val response = StringBuffer()

                var inputLine = it.readLine()
                while (inputLine != null) {
                    response.append(inputLine)
                    inputLine = it.readLine()
                }
                //println("Response : $response")

                res = response.toString()
            }
        }
        return res
    }


}