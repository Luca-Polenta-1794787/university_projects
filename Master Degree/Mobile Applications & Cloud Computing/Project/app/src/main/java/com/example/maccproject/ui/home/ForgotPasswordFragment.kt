package com.example.maccproject.ui.home

import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.navigation.fragment.findNavController
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentForgotPasswordBinding
import com.google.firebase.auth.ktx.auth
import com.google.firebase.ktx.Firebase

class ForgotPasswordFragment : Fragment() {

    private var _binding: FragmentForgotPasswordBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!
    private val tagLoginFragment = "LoginFragment"

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?): View {

        _binding = FragmentForgotPasswordBinding.inflate(inflater, container, false)

        binding.submitEmail.setOnClickListener {
            Firebase.auth.sendPasswordResetEmail(binding.email.text.toString())
                .addOnCompleteListener { task ->
                    if (task.isSuccessful) {
                        Log.d(tagLoginFragment, "Email sent.")
                        Toast.makeText(activity?.applicationContext,"Check your email to reset the password!", Toast.LENGTH_LONG).show()
                        findNavController().navigate(R.id.action_forgotPasswordFragment_to_loginFragment)
                    }
                    else{
                        Log.e(tagLoginFragment,"Email not sent:", task.exception)
                        Toast.makeText(activity?.applicationContext,"I could not send the email, check if you write the email correctly", Toast.LENGTH_LONG).show()
                    }
                }
        }

        return binding.root
    }

}