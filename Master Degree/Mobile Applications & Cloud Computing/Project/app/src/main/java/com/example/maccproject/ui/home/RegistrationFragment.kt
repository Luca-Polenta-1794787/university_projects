package com.example.maccproject.ui.home

import android.os.Bundle
import android.util.Log
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Toast
import androidx.fragment.app.Fragment
import androidx.navigation.NavController
import androidx.navigation.Navigation
import androidx.navigation.findNavController
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentRegistrationBinding
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.database.DatabaseReference
import com.google.firebase.database.FirebaseDatabase
import java.util.regex.Pattern

class RegistrationFragment : Fragment(){

    private var _binding: FragmentRegistrationBinding? = null

    //Firebase references
    private var mDatabaseReference: DatabaseReference? = null
    private var mDatabase: FirebaseDatabase? = null
    private var mAuth: FirebaseAuth? = null

    private val tagCreateAccountFragment = "CreateAccountFragment"

    //global variables
    private var email: String? = null
    private var password: String? = null
    private lateinit var navController: NavController

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    //Send verification email to the user
    private fun verifyEmail() {
        val mUser = mAuth!!.currentUser
        mUser!!.sendEmailVerification()
            .addOnCompleteListener(requireActivity()) { task ->
                if (task.isSuccessful) {
                    Log.d(tagCreateAccountFragment, "sendEmailVerification:ok")
                    Toast.makeText(activity?.applicationContext,
                        "Verification email sent to " + mUser.email,
                        Toast.LENGTH_LONG).show()
                } else {
                    Log.e(tagCreateAccountFragment, "sendEmailVerification:", task.exception)
                    Toast.makeText(activity?.applicationContext,
                        "Failed to send verification email.",
                        Toast.LENGTH_LONG).show()
                }
            }
    }

    private fun validateFields(email:String, password:String): Boolean{
        val EMAIL_ADDRESS_PATTERN = Pattern.compile(
            "[a-zA-Z0-9\\+\\.\\_\\%\\-\\+]{1,256}" +
                    "\\@" +
                    "[a-zA-Z0-9][a-zA-Z0-9\\-]{0,64}" +
                    "(" +
                    "\\." +
                    "[a-zA-Z0-9][a-zA-Z0-9\\-]{0,25}" +
                    ")+"
        )
        val PASSWORD_PATTERN = Pattern.compile("^(?=.*[a-z])(?=.*[A-Z])(?=.*[0-9])(?=\\S+$).{7,}")
        //Validity of email
        if (!EMAIL_ADDRESS_PATTERN.matcher(email).matches()){
            return false
        }
        //Validity of password
        if(!PASSWORD_PATTERN.matcher(password).matches()){
            return false
        }
        return true
    }

    private fun createNewAccount(){
        //get values from the registration form
        email = binding.registrationEmail.text.toString()
        password = binding.registrationPassword.text.toString()
        //validate textual data and eventually show error message
        if (!validateFields(email!!, password!!)) {
            Toast.makeText(activity?.applicationContext, "Enter all details and/or use a better password", Toast.LENGTH_LONG).show()
            Log.d(tagCreateAccountFragment, "Inputs from the user are not good enough!")
            return
        }
        //Firebase
        mAuth!!
            .createUserWithEmailAndPassword(email!!, password!!)
            .addOnCompleteListener(requireActivity()) { task ->
                if (task.isSuccessful) {
                    // Sign in success, update UI with the signed-in user's information
                    Log.d(tagCreateAccountFragment, "createUserWithEmail:success")
                    val userId = mAuth!!.currentUser!!.uid
                    //Send a verification email
                    verifyEmail()
                    //Go to the login page
                    if(navController.currentDestination?.id == R.id.registrationFragment) {
                        navController.navigate(R.id.action_registrationFragment_to_loginFragment)
                    }
                } else {
                    // If sign in fails, display a message to the user.
                    Log.w(tagCreateAccountFragment, "createUserWithEmail:failure", task.exception)
                    val toast = Toast.makeText(activity?.applicationContext, "Email already in use!",
                        Toast.LENGTH_LONG)
                    toast.show()
                }
            }
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        navController = Navigation.findNavController(view)
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?): View {

        _binding = FragmentRegistrationBinding.inflate(inflater, container, false)
        val root: View = binding.root
        mDatabase = FirebaseDatabase.getInstance("https://maccproject-10167-default-rtdb.europe-west1.firebasedatabase.app")
        mDatabaseReference = mDatabase!!.reference.child("Users")
        mAuth = FirebaseAuth.getInstance()
        //Register user
        binding.registrationButton.setOnClickListener{
            Log.d(tagCreateAccountFragment, "Clicked on registration button")
            createNewAccount()
        }
        //Go to login page
        binding.LoginLink.setOnClickListener { v: View ->
            v.findNavController().navigate(R.id.action_registrationFragment_to_loginFragment)
        }
        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}

