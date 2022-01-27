package com.example.maccproject.ui.home

import android.os.Bundle
import android.text.TextUtils
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.navigation.NavController
import androidx.navigation.Navigation
import androidx.navigation.findNavController
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentLoginBinding
import com.google.firebase.auth.FirebaseAuth

class LoginFragment : Fragment() {

    private var _binding: FragmentLoginBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    private val TAG = "LoginFragment"
    //global variables
    private var email: String? = null
    private var password: String? = null
    //Firebase references
    private var mAuth: FirebaseAuth? = null
    private lateinit var navController: NavController

    private fun loginUser() {
        email = binding.email.text.toString()
        password = binding.password.text.toString()
        if (!TextUtils.isEmpty(email) && !TextUtils.isEmpty(password)) {
            mAuth!!.signInWithEmailAndPassword(email!!, password!!)
                .addOnCompleteListener(requireActivity()) { task ->
                    if (task.isSuccessful) {
                        // Sign in success, update UI with signed-in user's information
                        val currentUser = mAuth!!.currentUser
                        if (currentUser != null) {
                            if(currentUser?.isEmailVerified()) {
                                Toast.makeText(
                                    activity?.applicationContext,
                                    "Logged in! May the force be with you.",
                                    Toast.LENGTH_LONG
                                ).show()
                                Log.d(
                                    TAG,
                                    "signInWithEmail, current user:: " + currentUser.uid + " " + currentUser.email
                                )
                                //Move to user page
                                navController.navigate(R.id.action_loginFragment_to_userFragment)
                            }
                            else{
                                //Email not verified
                                Log.e(TAG, "signInWithEmail: email not verified")
                                Toast.makeText(
                                    activity?.applicationContext,
                                    "You should verify your email before logging in",
                                    Toast.LENGTH_LONG
                                ).show()
                            }
                        } else {
                            Log.d(TAG, "current user is null")
                        }
                    } else {
                        // If sign in fails, display a message to the user.
                        Log.e(TAG, "signInWithEmail:failure", task.exception)
                        Toast.makeText(activity?.applicationContext, "Authentication failed.",
                            Toast.LENGTH_LONG).show()
                    }
                }
        } else {
            Toast.makeText(activity?.applicationContext, "Enter all details", Toast.LENGTH_LONG).show()
        }
    }
    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        navController = Navigation.findNavController(view)
    }
    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {

        _binding = FragmentLoginBinding.inflate(inflater, container, false)
        mAuth = FirebaseAuth.getInstance()
        //Login with email and password
        binding.loginBtn.setOnClickListener { loginUser() }

        //Go to registration page
        binding.RegisterLink.setOnClickListener { v: View ->
            v.findNavController().navigate(R.id.action_loginFragment_to_registrationFragment)
        }
        //Go to forgot password page
        binding.forgotPassword.setOnClickListener { v: View ->
            v.findNavController().navigate(R.id.action_loginFragment_to_forgotPasswordFragment)
        }

        return binding.root

    }

}