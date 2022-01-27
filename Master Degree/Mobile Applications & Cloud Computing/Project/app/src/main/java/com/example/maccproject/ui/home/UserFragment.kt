package com.example.maccproject.ui.home

import android.os.Bundle
import android.util.Log
import androidx.fragment.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import android.widget.Toast
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.navigation.NavController
import androidx.navigation.Navigation
import androidx.navigation.findNavController
import com.example.maccproject.R
import com.example.maccproject.databinding.FragmentUserBinding
import com.facebook.login.LoginManager
import com.google.android.material.bottomnavigation.BottomNavigationView
import com.google.firebase.auth.FirebaseAuth
import com.google.firebase.database.*

class UserFragment : Fragment() {

    private var _binding: FragmentUserBinding? = null
    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!
    //Firebase references
    private var mDatabaseReference: DatabaseReference? = null
    private var mDatabase: FirebaseDatabase? = null
    private var mAuth: FirebaseAuth? = null
    //UI elements
    private var tvEmail: TextView? = null
    private var quiz: TextView? = null
    private var game: TextView? = null

    private val tagUserFragment:String = "UserFragment"
    private lateinit var navController: NavController

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)
        navController = Navigation.findNavController(view)
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?): View {

        _binding = FragmentUserBinding.inflate(inflater, container, false)
        val root: View = binding.root
        //Show top and bottom action bars
        (activity as AppCompatActivity).supportActionBar?.show()
        val navView: BottomNavigationView? = activity?.findViewById(R.id.nav_view)
        navView?.visibility = View.VISIBLE
        //Hide back button
        if(activity is AppCompatActivity){
            (activity as AppCompatActivity?)?.supportActionBar?.setDisplayHomeAsUpEnabled(false)
            (activity as AppCompatActivity?)?.supportActionBar?.title = "User Info"
        }

        initialise()
        //Sign out
        binding.signout.setOnClickListener { v: View ->
            mAuth?.signOut()
            LoginManager.getInstance().logOut()
            v.findNavController().navigate(R.id.action_userFragment_to_navigation_home)
        }
        //Delete account
        binding.deleteUser.setOnClickListener{ v:View ->
            val builder = AlertDialog.Builder(requireActivity())
            builder.setMessage("Are you sure you want to Delete?")
                .setCancelable(false)
                .setPositiveButton("Yes") { dialog, id ->
                    // Delete user
                    deleteUser()
                    if(navController.currentDestination?.id == R.id.userFragment) {
                        Log.d(tagUserFragment, "Moving from: "+ navController.currentDestination?.id)
                        navController.navigate(R.id.action_userFragment_to_navigation_home)
                    }
                    else{
                        Log.d(tagUserFragment, "currentDestination id: "+ navController.currentDestination?.id)
                    }
                    Toast.makeText(activity?.applicationContext, "Account deleted", Toast.LENGTH_LONG).show()
                }
                .setNegativeButton("No") { dialog, id ->
                    // Dismiss the dialog
                    dialog.dismiss()
                }
            val alert = builder.create()
            alert.show()
        }

        return root
    }
    private fun initialise() {
        mDatabase = FirebaseDatabase.getInstance()
        mDatabaseReference = mDatabase!!.reference.child("Users")
        Log.d(tagUserFragment, "mDatabaseReference: $mDatabaseReference")
        mAuth = FirebaseAuth.getInstance()
        Log.d(tagUserFragment, "mAuth: $mAuth")
        tvEmail = binding.tvEmail
        quiz = binding.tvQuiz
        game = binding.tvGame
    }

    override fun onStart() {
        super.onStart()
        val currentUser = mAuth!!.currentUser
        if (currentUser != null) {
            Log.d(tagUserFragment, "Current user: " + currentUser?.uid.toString())
        } else {
            Log.d(tagUserFragment, "current user is null")
        }
        //Update interface
        tvEmail!!.text = currentUser?.email
        val quizRef = mDatabaseReference?.child(currentUser?.uid.toString())
        val valueEventListener = object : ValueEventListener {
            override fun onDataChange(dataSnapshot: DataSnapshot) {
                //Show the results for the quiz
                val quizValue = dataSnapshot.child("quiz").getValue(String::class.java)
                Log.d(tagUserFragment, "DataBaseGetQuizResult: $quizValue")
                if(quizValue==null){
                    quiz!!.text = "You have not done the quiz yet"
                }
                else {
                    quiz!!.text = quizValue
                }
                //Show the number of games the user won
                val gameValue = dataSnapshot.child("game").getValue(String::class.java)
                Log.d(tagUserFragment, "DataBaseGetQuizResult: $gameValue")
                if(gameValue==null){
                    game!!.text = "You have not played the game yet!"
                }
                else {
                    if (gameValue == "1") {
                        game!!.text = "$gameValue victory"
                    } else {
                        game!!.text = "$gameValue victories"
                    }
                }
            }

            override fun onCancelled(databaseError: DatabaseError) {
                Log.d("Data", databaseError.message) //Don't ignore errors!
            }
        }
        quizRef?.addValueEventListener(valueEventListener)

    }

    fun deleteUser(){
        val currentUser = mAuth!!.currentUser
        mAuth?.signOut()
        LoginManager.getInstance().logOut()
        currentUser!!.delete()
            .addOnCompleteListener { task ->
                if (task.isSuccessful) {
                    Log.d(tagUserFragment, "User account deleted.")
                }
            }
    }

}