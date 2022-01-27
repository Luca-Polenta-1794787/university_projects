package com.example.maccproject.ui.home

import android.content.Intent
import android.os.Bundle
import android.util.Log
import android.view.*
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity
import androidx.fragment.app.Fragment
import androidx.navigation.NavController
import androidx.navigation.Navigation
import androidx.navigation.findNavController
import androidx.navigation.fragment.findNavController
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentHomeBinding
import com.facebook.*
import com.facebook.login.LoginManager
import com.facebook.login.LoginResult
import com.google.android.material.bottomnavigation.BottomNavigationView
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.auth.FacebookAuthProvider
import com.google.firebase.auth.ktx.auth
import com.google.firebase.ktx.Firebase

class HomeFragment : Fragment() {

    private var _binding: FragmentHomeBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!
    private val tagHomeFragment = "HomeFragment"
    private lateinit var navController: NavController
    private lateinit var auth: FirebaseAuth
    lateinit var callbackManager: CallbackManager

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        callbackManager.onActivityResult(requestCode, resultCode, data)
        super.onActivityResult(requestCode, resultCode, data)
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        navController = Navigation.findNavController(view)
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle? ): View {

        _binding = FragmentHomeBinding.inflate(inflater, container, false)
        val root: View = binding.root
        auth = Firebase.auth

        //Check if user is already logged in with facebook
        if (isLoggedIn()) {
            // Show the user fragment
            findNavController().navigate(R.id.action_navigation_home_to_userFragment)
        }

        //go to login page
        binding.loginBtn.setOnClickListener { v: View ->
            v.findNavController().navigate(R.id.action_navigation_home_to_loginFragment)
        }
        //go to registration page
        binding.registerButton.setOnClickListener { v: View ->
            v.findNavController().navigate(R.id.action_navigation_home_to_registrationFragment)
        }
        //Facebook sign in
        callbackManager = CallbackManager.Factory.create()
        binding.loginButton.fragment = this
        binding.loginButton.setOnClickListener {
            LoginManager.getInstance().logInWithReadPermissions(this, listOf("public_profile", "email"))
        }

        // Callback registration
        binding.loginButton.registerCallback(callbackManager, object :
            FacebookCallback<LoginResult?> {
            override fun onSuccess(loginResult: LoginResult?) {
                Log.d(tagHomeFragment, "Success Login")
                handleFacebookAccessToken(loginResult!!.accessToken)
            }

            override fun onCancel() {
                Toast.makeText(activity?.applicationContext, "Login Cancelled", Toast.LENGTH_LONG).show()
            }

            override fun onError(exception: FacebookException) {
                Toast.makeText(activity?.applicationContext, exception.message, Toast.LENGTH_LONG).show()
            }
        })

        return root
    }

    override fun onResume(){
        super.onResume()
        //Hide top and bottom action bars
        (activity as AppCompatActivity).supportActionBar?.hide()
        val navView: BottomNavigationView? = activity?.findViewById(R.id.nav_view)
        navView?.visibility = View.GONE
    }
    override fun onStart() {
        super.onStart()
        val auth = Firebase.auth
        // Check if user is signed in (non-null) and update UI accordingly.
        val currentUser = auth.currentUser
        if(currentUser!=null) {
            Log.d(tagHomeFragment, "There is already a user")
            this.findNavController().navigate(R.id.action_navigation_home_to_userFragment)
        }
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }

    fun isLoggedIn(): Boolean {
        val accessToken = AccessToken.getCurrentAccessToken()
        val isLoggedIn = accessToken != null && !accessToken.isExpired
        return isLoggedIn
    }

    private fun handleFacebookAccessToken(token: AccessToken) {
        Log.d(tagHomeFragment, "handleFacebookAccessToken:$token")

        val credential = FacebookAuthProvider.getCredential(token.token)
        auth.signInWithCredential(credential)
            .addOnCompleteListener(requireActivity()) { task ->
                if (task.isSuccessful) {
                    // Sign in success, update UI with the signed-in user's information
                    Log.d(tagHomeFragment, "signInWithCredentialFacebook:success")
                    val currentUser = auth.currentUser
                    if (currentUser != null) {
                        Toast.makeText(activity?.applicationContext, "Logged in! May the force be with you.", Toast.LENGTH_LONG).show()
                        Log.d(tagHomeFragment, "signInWithFacebook, current user:: " + currentUser.uid + " " + currentUser.email)
                        if(navController.currentDestination?.id == R.id.navigation_home) {
                            navController.navigate(R.id.action_navigation_home_to_userFragment)
                        }
                        else{
                            Log.d(tagHomeFragment, "currentDestination id: "+ navController.currentDestination?.id)
                        }
                    } else {
                        Log.d(tagHomeFragment, "current user is null")
                    }
                } else {
                    // If sign in fails, display a message to the user.
                    Log.w(tagHomeFragment, "signInWithCredential:failure", task.exception)
                    Toast.makeText(activity?.applicationContext, "Authentication failed.",
                        Toast.LENGTH_LONG).show()
                }
            }
    }

}